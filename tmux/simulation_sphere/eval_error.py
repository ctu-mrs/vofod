#!/usr/bin/env python3

import rospy
import tf2_geometry_msgs
import tf2_ros
import geometry_msgs
import rosbag
import tf_bag
import os
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import pickle
import copy
import csv
from sympy.stats import Nakagami, density, E, variance, cdf

from scipy.special import gamma
from scipy.spatial import transform as sptf
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation


# #{ helper functions

def quat2xyzrot(quat):
    # implementation from PlotJuggler
    q_x = quat[0]
    q_y = quat[1]
    q_z = quat[2]
    q_w = quat[3]

    quat_norm2 = (q_w * q_w) + (q_x * q_x) + (q_y * q_y) + (q_z * q_z)
    if np.abs(quat_norm2 - 1.0) > 0.0:
        mult = 1.0 / np.sqrt(quat_norm2)
        q_x *= mult
        q_y *= mult
        q_z *= mult
        q_w *= mult

    # roll (x-axis rotation)
    sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
    cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q_w * q_y - q_z * q_x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(M_PI_2, sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    # yaw (z-axis rotation)
    siny_cosp = 2 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    angs = np.array(( roll, pitch, yaw ))
    return angs


def load_quat(msg):
    if hasattr(msg, "orientation"):
        return np.array((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
    elif hasattr(msg, "pose"):
        return load_quat(msg.pose)
    else:
        print("Unknown message type: {}".msg)
        exit(1)


def load_avels(msg):
    if hasattr(msg, "angular_velocity"):
        return np.array((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
    elif hasattr(msg, "twist"):
        return load_avels(msg.twist)
    elif hasattr(msg, "velocity"):
        return load_avels(msg.velocity)
    elif hasattr(msg, "angular"):
        return np.array((msg.angular.x, msg.angular.y, msg.angular.z))
    else:
        print("Unknown message type: {}".msg)
        exit(1)


def slerp_rotations(to_times, from_times, from_rots):
        slerp = Slerp(from_times, from_rots)

        # ensure that the interpolation times are within the from_times limits
        time_greater = to_times > from_times[0]
        time_smaller = to_times < from_times[-1]
        within_time = time_greater * time_smaller
        slerp_times = to_times[within_time]

        # do the actual interpolation using SciPy
        slerped = slerp(slerp_times)
        return slerped, within_time


def rots_to_xyz(rotations):
    return np.array([quat2xyzrot(R.as_quat()) for R in rotations])
    

# #} end of helper methods


# #{ load_rosbag function

def load_rosbag(bag, obs_uav, params):
    ang_topic = '/'+obs_uav+'/'+params["ang_topic"]
    avel_topic = '/'+obs_uav+'/'+params["avel_topic"]
    gt_ang_topic = '/'+obs_uav+'/'+params["gt_ang_topic"]
    gt_avel_topic = '/'+obs_uav+'/'+params["gt_avel_topic"]

    # primary_topic = None
    # if params["primary_topic"] == "ang":
    #     primary_topic = ang_topic
    # elif params["primary_topic"] == "avel":
    #     primary_topic = avel_topic
    # elif params["primary_topic"] == "gt_ang_topic":
    #     primary_topic = gt_ang_topic
    # elif params["primary_topic"] == "gt_avel_topic":
    #     primary_topic = gt_avel_topic
    # else:
    #     print("Unknown primary topic specified: {}!".format(params["primary_topic"]))
    #     exit(1)

    topics = (ang_topic, avel_topic, gt_ang_topic, gt_avel_topic)
    n_msgs = 0
    n_gt_rots = 0
    for topic, msg, t in bag.read_messages(topics=topics):
        n_msgs += 1
        if topic == gt_ang_topic:
            n_gt_rots += 1

    print("Total {} messages, processing.".format(n_msgs))
    ret = dict()
    ret["times"] = np.nan*np.ones((n_msgs,))
    ret["angs"] = np.nan*np.ones((n_msgs, 3))
    ret["avels"] = np.nan*np.ones((n_msgs, 3))
    ret["gt_angs"] = np.nan*np.ones((n_msgs, 3))
    ret["gt_avels"] = np.nan*np.ones((n_msgs, 3))

    gt_rot_times = list()
    gt_rots = list()

    print("Loading topics {}".format(topics))
    it = 0
    for topic, msg, t in bag.read_messages(topics=topics):

        ## | ----------------- parse the time or stamp ---------------- |
        if hasattr(msg, "header"):
            ret["times"][it] = msg.header.stamp.to_sec()
        else:
            print("Message has no header and thus no timestamp!")
            exit(1)

        ## | -------------------- parse the message ------------------- |
        if topic == ang_topic:
            ret["angs"][it, :] = quat2xyzrot(load_quat(msg))

        if topic == gt_ang_topic:
            gt_rot_times.append(ret["times"][it])
            R = Rotation(load_quat(msg))
            gt_rots.append(R)

        if topic == avel_topic:
            ret["avels"][it, :] = load_avels(msg)

        if topic == gt_avel_topic:
            ret["gt_avels"][it, :] = load_avels(msg)

        it += 1

    # cut off invalid samples
    for key, val in ret.items():
        ret[key] = ret[key][:it]

    print("Loaded {}/{} total messages".format(it, n_msgs))
    print("Loaded {} angle messages on topic {}".format(np.sum(~np.isnan(ret["angs"]).any(axis=1)), ang_topic))
    print("Loaded {} velocity messages on topic {}".format(np.sum(~np.isnan(ret["avels"]).any(axis=1)), avel_topic))
    print("Loaded {} angle messages on topic {}".format(len(gt_rots), gt_ang_topic))
    print("Loaded {} velocity messages on topic {}".format(np.sum(~np.isnan(ret["gt_avels"]).any(axis=1)), gt_avel_topic))

    n_msgs = it

    ## | -------------- Sort the loaded data by time -------------- |
    sort_idcs = np.argsort(ret["times"])
    for key, val in ret.items():
        if not hasattr(val, 'shape') or len(val.shape) == 1:
            ret[key] = val[sort_idcs]
        else:
            ret[key] = val[sort_idcs, :]

    gt_rot_sort_idcs = np.argsort(gt_rot_times)
    gt_rot_times = np.array(gt_rot_times)[gt_rot_sort_idcs]
    gt_rots = Rotation.concatenate(np.array(gt_rots)[gt_rot_sort_idcs])

    dt = 0

    if params["preprocessing"] == "optimize_time":
        ax = params["ang_axis"]
        c_step = 0.01
        dt_step = 1e-2
        angs_valid = ~np.isnan(ret["angs"]).any(axis=1)
        for tit in range(100):
            cur_gt_rot_times = gt_rot_times + dt
            slerped_gt_rots, slerp_within_time = slerp_rotations(ret["times"], cur_gt_rot_times, gt_rots)

            gt_angs_tfit = np.zeros_like(ret["angs"])
            gt_angs_tfit[slerp_within_time] = rots_to_xyz(slerped_gt_rots)
            
            valid = angs_valid * ~np.isnan(gt_angs_tfit).any(axis=1)
            eangs = np.abs(ret["angs"][valid, ax] - gt_angs_tfit[valid, ax])
            avg_eang = np.mean(eangs)
            print("dt: {}s, err: {}rad".format(dt, avg_eang))

            # NEGATIVE DT
            gt_rot_times_n = cur_gt_rot_times - dt_step/2
            slerped_gt_rots, slerp_within_time = slerp_rotations(ret["times"], gt_rot_times_n, gt_rots)

            gt_angs_tfit = np.zeros_like(ret["angs"])
            gt_angs_tfit[slerp_within_time] = rots_to_xyz(slerped_gt_rots)
            
            valid = angs_valid * ~np.isnan(gt_angs_tfit).any(axis=1)
            eangs = np.abs(ret["angs"][valid, ax] - gt_angs_tfit[valid, ax])
            avg_eang_n = np.mean(eangs)

            # POSITIVE DT
            gt_rot_times_p = cur_gt_rot_times + dt_step/2
            slerped_gt_rots, slerp_within_time = slerp_rotations(ret["times"], gt_rot_times_p, gt_rots)

            gt_angs_tfit = np.zeros_like(ret["angs"])
            gt_angs_tfit[slerp_within_time] = rots_to_xyz(slerped_gt_rots)
            
            valid = angs_valid * ~np.isnan(gt_angs_tfit).any(axis=1)
            eangs = np.abs(ret["angs"][valid, ax] - gt_angs_tfit[valid, ax])
            avg_eang_p = np.mean(eangs)

            diff = (avg_eang_p - avg_eang_n) / dt_step
            dt += - c_step * diff

            print("diff: {}rad/s, new dt: {}s".format(diff, dt))

    ## | ----------- Interpolate the missing g.t. values ---------- |
    slerped_gt_rots, slerp_within_time = slerp_rotations(ret["times"], gt_rot_times+dt, gt_rots)
    ret["gt_angs"][slerp_within_time] = rots_to_xyz(slerped_gt_rots)

    if params["preprocessing"] == "optimize_time":
        eangs = np.abs(ret["angs"][:, ax] - ret["gt_angs"][:, ax])
        avg_eang = np.mean(eangs)
        print("dt: {}s, err: {}rad".format(dt, avg_eang))
        
    avels_invalid = np.isnan(ret["gt_avels"]).any(axis=1)
    avels_valid = ~avels_invalid
    for it in range(3):
        x = ret["times"]
        xp = ret["times"][avels_valid]
        fp = ret["gt_avels"][avels_valid, it]
        ret["gt_avels"][:, it] = np.interp(x, xp, fp)

    return ret

# #} end of load_rosbag function


# #{ load_and_process_rosbag function

def load_and_process_rosbag(bag_fpath: str, obs_uav: str, start_t: float, params):

    bag_fname = os.path.basename(bag_fpath)
    pkl_fname = "cache/{}_{}_{}.pkl".format(bag_fname, params["cache_prefix"], params["preprocessing"])

    if os.path.isfile(pkl_fname):
        print("Using pickle file '{}'...".format(pkl_fname))
        with open(pkl_fname, "rb") as fhandle:
            loaded = pickle.load(fhandle)

    else:
        print("Opening bag file '{}'...".format(bag_fpath))
        with rosbag.Bag(bag_fpath, 'r') as bag:
            print("Bag file opened.")
            loaded = load_rosbag(bag, obs_uav, params)

            os.makedirs(os.path.dirname(pkl_fname), exist_ok=True)
            with open(pkl_fname, mode="wb") as fhandle:
                pickle.dump(loaded, fhandle)

    if params["preprocessing"] == "predict_angle":
        loaded["angs"] += params["predict_angle_dt"]*loaded["avels"]

    loaded["times"] = loaded["times"] - loaded["times"][0]
    invalid0 = np.isnan(loaded["times"])
    invalid1 = np.isnan(loaded["gt_avels"]).any(axis=1)
    invalid2 = np.isnan(loaded["gt_angs"]).any(axis=1)
    invalid3 = np.isnan(loaded["avels"]).any(axis=1)
    invalid4 = np.isnan(loaded["angs"]).any(axis=1)
    valid = ~(invalid0 + invalid1 + invalid2 + invalid3 + invalid4)

    loaded["valid"] = valid
    # start_t = -np.inf
    end_t = np.inf

    invalid_ts = np.logical_or(loaded["times"] < start_t, loaded["times"] > end_t)
    for key, val in loaded.items():
        if not hasattr(val, 'shape') or len(val.shape) == 1:
            loaded[key][invalid_ts] = np.nan
        else:
            loaded[key][invalid_ts, :] = np.nan

    return loaded

# #} end of load_and_process_rosbag function


# #{ load_and_process_rosbags function

def load_and_process_rosbags(bags, params):
    ret = dict()

    for bag in bags:
        processed = load_and_process_rosbag(bag["fpath"], bag["obs_uav"], bag["skip_t"], params)

        if len(ret) == 0:
            ret = processed
        else:
            processed["times"] += ret["times"][-1]
            for key, val in processed.items():
                if not hasattr(val, 'shape'):
                    ret[key] = ret[key] + processed[key]
                elif len(val.shape) == 1:
                    ret[key] = np.hstack((ret[key], processed[key]))
                else:
                    ret[key] = np.vstack((ret[key], processed[key]))

    return ret

# #} end of load_and_process_rosbags function


if __name__ == "__main__":
    params = dict()
    params["experiments"] = "sim"
    ang_axis = 1 # pitch

    # params["cache_prefix"] = "mavros"
    # params["ang_topic"] = "mavros/imu/data"
    # params["avel_topic"] = "mavros/imu/data"
    # params["predict_angle_dt"] = 0.01

    params["cache_prefix"] = "estimator"
    params["ang_topic"] = "estimation_manager/rtk/odom"
    params["avel_topic"] = "estimation_manager/rtk/odom"
    params["predict_angle_dt"] = 0.029

    params["gt_ang_topic"] = "ground_truth"
    params["gt_avel_topic"] = "ground_truth"
    params["preprocessing"] = "predict_angle"
    # params["preprocessing"] = "optimize_time"
    # params["preprocessing"] = "no"
    params["ang_axis"] = ang_axis
    do_plot = True

    bags = None
    if params["experiments"] == "sim":
        ofile_name = "simulation_noisy"
        bags = (
            {"fpath": "/home/matous/bag_files/eagle/sphere_test/filtered.bag", "obs_uav": "uav1", "skip_t": 0},
        )

    data = load_and_process_rosbags(bags, params)

    v = data["valid"]
    times = data["times"][v]
    vels = data["avels"][v, ang_axis]
    angs = data["angs"][v, ang_axis]
    gt_angs = data["gt_angs"][v, ang_axis]
    eangs = gt_angs - angs

    bins = np.linspace(np.min(vels), np.max(vels), 20)
    bin_ctrs = bins + (bins[1]-bins[0])/2
    bin_ctrs = bin_ctrs[:-1]
    # make a zero array 
    stddevs = np.zeros_like(bin_ctrs)
    means = np.zeros_like(bin_ctrs)
    for it in range(len(bins)-1):
        binmin = bins[it]
        binmax = bins[it+1]
        bin_idcs = np.logical_and(vels >= binmin, vels < binmax)
        bin_errs = eangs[bin_idcs]
        means[it] = np.mean(bin_errs)
        stddevs[it] = np.std(bin_errs)

    plt.title("Estimation error from {} with {} preprocessing".format(params["cache_prefix"], params["preprocessing"]))
    plt.plot(bin_ctrs, means, label="means")
    plt.plot(bin_ctrs, stddevs, label="std. devs")
    plt.xlabel("angular velocity [rad/s]")
    plt.ylabel("angular error [rad]")
    plt.legend()
    plt.show()


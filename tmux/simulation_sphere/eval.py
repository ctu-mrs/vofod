#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
from mrs_msgs.msg import UavState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as spR


def msg2xyzrot(quat_msg):
    # implementation from PlotJuggler
    q_x = quat_msg.x
    q_y = quat_msg.y
    q_z = quat_msg.z
    q_w = quat_msg.w

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

    ang = np.array(( roll, pitch, yaw ))
    return ang


ang = None
ang_vel = None
ang_gt = None
ang_vel_gt = None

def cbk_state(msg):
    global evel_over_vel, ang_vel_gt
    global eang_over_vel, ang_gt
    global ang, ang_vel

    # ang_msg = msg.pose.pose.orientation
    ang_msg = msg.orientation
    ang = msg2xyzrot(ang_msg)

    # ang_vel_msg = msg.twist.twist.angular
    ang_vel_msg = msg.angular_velocity
    ang_vel = np.array((ang_vel_msg.x, ang_vel_msg.y, ang_vel_msg.z))

    if ang_vel_gt is not None and ang_vel is not None:
        evel = ang_vel_gt - ang_vel
        evel_over_vel = np.append(evel_over_vel, [[ang_vel[1], evel[1]]], axis=0)

    if ang_gt is not None and ang is not None:
        eang = ang_gt - ang
        eang_over_vel = np.append(eang_over_vel, [[ang_vel[1], eang[1]]], axis=0)


def cbk_gt_state(msg):
    global evel_over_vel, ang_vel
    global eang_over_vel, ang
    global ang_gt, ang_vel_gt

    ang_msg = msg.pose.pose.orientation
    ang_gt = msg2xyzrot(ang_msg)

    ang_vel_msg = msg.twist.twist.angular
    ang_vel_gt = np.array((ang_vel_msg.x, ang_vel_msg.y, ang_vel_msg.z))


def cbk_pc(msg):
    global tf_buffer
    global ang_vel
    global dist_over_vel
    to_frame = "uav1/rtk_origin"
    transform = tf_buffer.lookup_transform(to_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
    pc_tfd = do_transform_cloud(msg, transform)
    pts = list()
    for p in pc2.read_points(pc_tfd, field_names = ("x", "y", "z"), skip_nans=True):
        pts.append(p)

    if len(pts) > 0:
        pts = np.array(pts)
        ctr = np.mean(pts, axis=0)
        gt_ctr = np.array((25, 0, 10))
        dist = np.linalg.norm(ctr - gt_ctr)
        ang_vel_y = ang_vel[1]
        dist_over_vel = np.append(dist_over_vel, [[ang_vel_y, dist]], axis=0)


if __name__ == "__main__":
    global tf_buffer
    global dist_over_vel
    global evel_over_vel
    global eang_over_vel
    rospy.init_node("main", anonymous=True)

    dist_over_vel = np.empty(shape=(0, 2))
    evel_over_vel = np.empty(shape=(0, 2))
    eang_over_vel = np.empty(shape=(0, 2))

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber("/uav1/pcl_filter/points_processed", PointCloud2, cbk_pc, queue_size = 1)
    # rospy.Subscriber("/uav1/estimation_manager/rtk/odom", Odometry, cbk_state, queue_size = 1)
    # rospy.Subscriber("/uav1/estimation_manager/ground_truth/odom", Odometry, cbk_gt_state, queue_size = 1)
    rospy.Subscriber("/uav1/mavros/imu/data", Imu, cbk_state, queue_size = 1)
    rospy.Subscriber("/uav1/ground_truth", Odometry, cbk_gt_state, queue_size = 1)

    N_max = 10000
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        N = len(eang_over_vel)
        if N >= N_max:
            break
        else:
            print("{}/{} samples".format(N, N_max))
            r.sleep()

    vels = eang_over_vel[:, 0]
    eangs = eang_over_vel[:, 1]
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

    plt.plot(bin_ctrs, means, label="means")
    plt.plot(bin_ctrs, stddevs, label="std. devs")
    plt.xlabel("angular velocity [rad/s]")
    plt.ylabel("angular error [rad]")
    plt.legend()
    plt.show()

    # plt.subplot(3, 1, 1)
    # plt.plot(doa[:, 0], doa[:, 1], 'go', label="sphere center RMSE")
    # plt.xlabel("RTK angular velocity around Y axis [rad/s]")
    # plt.ylabel("RMSE sphere error [m]")

    # plt.subplot(3, 1, 2)
    # plt.plot(evel_over_vel[:, 0], evel_over_vel[:, 1], 'rx', label="velocity error")
    # plt.xlabel("RTK angular velocity around Y axis [rad/s]")
    # plt.ylabel("g.t. angular velocity error [rad/s]")

    # plt.subplot(3, 1, 3)
    # plt.plot(eang_over_vel[:, 0], eang_over_vel[:, 1], 'bx', label="angular error")
    # plt.xlabel("RTK angular velocity around Y axis [rad/s]")
    # plt.ylabel("g.t. orientation error [rad]")

    # plt.show()

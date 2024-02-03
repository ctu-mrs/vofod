#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
from mrs_msgs.msg import UavState
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt


ang = None
ang_vel = None
def cbk_state(msg):
    global ang, ang_vel

    # ang_msg = msg.pose.orientation
    # ang = np.array((ang_vel_cur_msg.x, ang_vel_cur_msg.y, ang_vel_cur_msg.z))

    ang_vel_msg = msg.velocity.angular
    ang_vel = np.array((ang_vel_msg.x, ang_vel_msg.y, ang_vel_msg.z))

    prev_state = msg


def cbk_gt_state(msg):
    global evel_over_vel
    global ang_vel

    if ang_vel is None:
        return

    ang_vel_gt_msg = msg.velocity.angular
    ang_vel_gt = np.array((ang_vel_gt_msg.x, ang_vel_gt_msg.y, ang_vel_gt_msg.z))

    ang_vel_gt_msg = msg.velocity.angular
    ang_vel_gt = np.array((ang_vel_gt_msg.x, ang_vel_gt_msg.y, ang_vel_gt_msg.z))
    evel = ang_vel_gt - ang_vel
    evel_over_vel = np.append(evel_over_vel, [[ang_vel[1], evel[1]]], axis=0)



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
        print(ang_vel, dist)
        ang_vel_y = ang_vel[1]
        dist_over_vel = np.append(dist_over_vel, [[ang_vel_y, dist]], axis=0)

    else:
        print("no points received")


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
    rospy.Subscriber("/uav1/estimation_manager/rtk/uav_state", UavState, cbk_state, queue_size = 1)
    rospy.Subscriber("/uav1/estimation_manager/ground_truth/uav_state", UavState, cbk_gt_state, queue_size = 1)

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        r.sleep()
        doa = dist_over_vel
        plt.subplot(2, 1, 1)
        plt.plot(doa[:, 0], doa[:, 1], 'go', label="sphere center RMSE")
        plt.xlabel("RTK angular velocity around Y axis [rad/s]")
        plt.ylabel("RMSE sphere error [m]")
        plt.legend()
        plt.subplot(2, 1, 2)
        plt.plot(evel_over_vel[:, 0], evel_over_vel[:, 1], 'rx', label="velocity error")
        plt.xlabel("RTK angular velocity around Y axis [rad/s]")
        plt.ylabel("angular velocity error [rad/s]")
        plt.legend()
        plt.show()

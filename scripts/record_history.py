#!/usr/bin/env python

# FILE: record_history.py
# ------------------------
# Record historical velocities and acceleration profiles

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from pathlib import Path
import os

vel_hist = []
acc_hist = []
prev_vel = np.array([0, 0, 0])
first_time = True
t = 0
t_first = 0
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def odomCB(odom):
	global prev_vel, vel_hist, prev_time, curr_time, first_time, t, t_first
	curr_time = rospy.get_rostime()
	if (first_time):
		t = 0
	else:
		if (t == 0):
			t_first = (curr_time - prev_time).to_sec()
		t += (curr_time - prev_time).to_sec()
		if ((curr_time - prev_time).to_sec() <= 0.1):
			return

	quat = [odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z]
	rot = quaternion_rotation_matrix(quat)

	vel_body = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z])
	vel_world = rot @ vel_body

	rospy.loginfo("Current velocity: %s %s %s", vel_world[0], vel_world[1], vel_world[2])

	if (first_time):
		first_time = False
		prev_time = curr_time
		prev_vel = vel_world
		rospy.sleep(rospy.Duration(0.1))
		return

	dt = (curr_time - prev_time).to_sec()
	acc_world = (vel_world - prev_vel)/dt
	prev_time = curr_time
	prev_vel = vel_world
	rospy.loginfo("Current acceleration: %s %s %s", acc_world[0], acc_world[1], acc_world[2])
	vel_world_with_time = np.insert(vel_world, 0, t-t_first)
	acc_world_with_time = np.insert(acc_world, 0, t-t_first)
	vel_hist.append(vel_world_with_time)
	acc_hist.append(acc_world_with_time)

	# rospy.sleep(rospy.Duration(0.1))


def main():
	rospy.init_node("record_history_node", anonymous=True)
	rospy.Subscriber("/mavros/local_position/odom", Odometry, odomCB)
	rospy.spin()


	save_path = Path(__file__).parent.parent.absolute()
	save_path = os.path.join(save_path, "log")

	np.savetxt(os.path.join(save_path, "vel_hist.txt"), vel_hist)
	np.savetxt(os.path.join(save_path, "acc_hist.txt"), acc_hist)


if __name__ == "__main__":
	main()
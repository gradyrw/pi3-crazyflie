#!/usr/bin/env python

import rospy
import numpy as np
from pi3_crazyflie_pkg.msg import rpyt, angles, acc, quadrotor_state, full_quadrotor_state
import rosbag
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import os.path
import datetime


class GoalUpdater:

    def __init__(self):
        self.goal = np.zeros(9)
        self.goal[2] = -1.75
        rospy.init_node("goal_node", anonymous=True)
        rospy.Subscriber("vicon/goalWand/goal_state", TransformStamped, self.update_goal) 
        self.pub = rospy.Publisher("goal_state", quadrotor_state)

    def update_goal(self, transform_stp):
        trans = transform_stp.transform
        adj_trans_x = (trans.translation.x) - 1.0
        adj_trans_y = -(trans.translation.y)
        adj_trans_z = -(trans.translation.z) 
        self.goal[0] = adj_trans_x
        self.goal[1] = adj_trans_y
        self.goal[2] = adj_trans_z -.5

if __name__ == "__main__":
    vicon_goal = GoalUpdater()
    r = rospy.Rate(10)
    msg = quadrotor_state()
    #Add meta-data to the first rosbag line
    stri = String()
    stri.data = "Alpha: " + str(rospy.get_param('ALPHA')) + " LAG: " + str(rospy.get_param('LAG')) + " VARS: " + str(rospy.get_param('VARS'))
    d = datetime.datetime.now()
    path = '/home/cudauser/irobot_quad/quad_cat/logging/'
    name = str(d) + "__" + "goal_node__" + "goal_state"
    full_name = os.path.join(path, name + ".bag")
    bag = rosbag.Bag(full_name, 'w')
    bag.write('metadata', stri)
    while not rospy.is_shutdown():
        #Create limited quadrotor state message for control
        msg.x = vicon_goal.goal[0]
        msg.y = vicon_goal.goal[1]
        msg.z = vicon_goal.goal[2]
        msg.roll = vicon_goal.goal[3]
        msg.pitch = vicon_goal.goal[4]
        msg.yaw = vicon_goal.goal[5]
        msg.x_dot = vicon_goal.goal[6]
        msg.y_dot = vicon_goal.goal[7]
        msg.z_dot = vicon_goal.goal[8]
        vicon_goal.pub.publish(msg)
        bag.write('goal_state', msg)
        r.sleep()
    bag.close()

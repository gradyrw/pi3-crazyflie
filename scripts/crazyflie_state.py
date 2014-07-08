#!/usr/bin/env python

import rospy
import numpy as np
from pi3_crazyflie_pkg.msg import rpyt, angles, acc, quadrotor_state, full_quadrotor_state
import rosbag
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from math import atan2, asin
import os.path
import datetime

class Sensor_Union:

    def __init__(self):
        self.state = np.zeros(13)
        self.full_state = np.zeros(17)
        self.vicon_eulers = np.zeros(3)
        rospy.init_node("transfer_node", anonymous=True)
        rospy.Subscriber("angle_sensors", angles, self.update_angles)
        rospy.Subscriber("control", rpyt, self.update_controls)
        rospy.Subscriber("vicon/crazyFlieWithMast/MainBody", TransformStamped, self.update_pos_vel) 
        self.pub = rospy.Publisher("state", quadrotor_state)

    def update_angles(self, angs):
        self.state[3] = angs.roll
        self.state[4] = angs.pitch
        self.state[5] = angs.yaw
        #Update full state as well
        self.full_state[9] = .8*self.full_state[9] + .2*(angs.roll - self.full_state[3])/.02 
        self.full_state[10] = .8*self.full_state[10] + .2*(angs.pitch - self.full_state[4])/.02
        self.full_state[11] = .8*self.full_state[11] + .2*(angs.yaw - self.full_state[5])/.02
        self.full_state[3] = angs.roll
        self.full_state[4] = angs.pitch
        self.full_state[5] = angs.yaw
        self.full_state[16] = angs.bat

    def update_controls(self, controls):
        #Update full state
        self.full_state[12] = controls.roll_rate
        self.full_state[13] = controls.pitch_rate
        self.full_state[14] = controls.yaw_rate
        self.full_state[15] = controls.thrust

    def update_pos_vel(self, transform_stp):
        trans = transform_stp.transform
        adj_trans_x = (trans.translation.x) -1.0
        adj_trans_y = -(trans.translation.y)
        adj_trans_z = (-trans.translation.z) 
        #/vicon/markers is published at 119.99 hz
        # newVelocity = alpha*Old + (1-alpha)*New
        self.state[6] = 0.0*self.state[6] + 1.0*(adj_trans_x - self.state[0]) * 119.99
        self.state[7] = 0.0*self.state[7] + 1.0*(adj_trans_y - self.state[1]) * 119.99
        self.state[8] = 0.0*self.state[8] + 1.0*(adj_trans_z - self.state[2]) * 119.99
        #Update position
        self.state[0] = adj_trans_x
        self.state[1] = adj_trans_y
        self.state[2] = adj_trans_z
        #Update logger state as well
        self.full_state[0:3] = self.state[0:3]
        self.full_state[6:9] = self.state[6:9]
        #Update vicon euler angles
        q0 = trans.rotation.x
        q1 = trans.rotation.y
        q2 = trans.rotation.z
        q3 = trans.rotation.w
        self.vicon_eulers[0] = atan2(2*q2*q3 + 2*q0*q1, q3**2 - q2**2 - q1**2 + q0**2)
        self.vicon_eulers[1] = -asin(2*q1*q3 - 2*q0*q2)
        self.vicon_eulers[2] = atan2(2*q1*q2 + 2*q0*q3, q1**2 + q0**2 - q3**2 - q2**2)
        

if __name__ == "__main__":
    rob = Sensor_Union()
    r = rospy.Rate(50) #50 hz
    count = 0
    msg = quadrotor_state()
    logger_msg = full_quadrotor_state()
    #Add meta-data to the first rosbag line
    stri = String()
    stri.data = "Alpha: " + str(rospy.get_param('ALPHA')) + " LAG: " + str(rospy.get_param('LAG')) + " VARS: " + str(rospy.get_param('VARS'))
    d = datetime.datetime.now()
    path = '/home/cudauser/irobot_quad/quad_cat/logging/'
    name = str(d) + "__" + "transfer_node__" + "full_quad_state"
    full_name = os.path.join(path, name + ".bag")
    bag = rosbag.Bag(full_name, 'w')
    bag.write('metadata', stri)
    print "got here"
    #str = "Alpha: " + str(alpha) + " Lag: " + str(lag) + " Var: " + str(var) 
    #metadata = String(data= "my metadata")
    while not rospy.is_shutdown():
        #Create limited quadrotor state message for control
        msg.x = rob.state[0]
        msg.y = rob.state[1]
        msg.z = rob.state[2]
        msg.roll = rob.state[3]
        msg.pitch = rob.state[4]
        msg.yaw = rob.state[5]
        msg.x_dot = rob.state[6]
        msg.y_dot = rob.state[7]
        msg.z_dot = rob.state[8]
        #Full quadrotor state message for logging
        logger_msg.x = rob.full_state[0]
        logger_msg.y = rob.full_state[1]
        logger_msg.z = rob.full_state[2]
        logger_msg.roll = rob.full_state[3]
        logger_msg.pitch = rob.full_state[4]
        logger_msg.yaw = rob.full_state[5]
        logger_msg.x_dot = rob.full_state[6]
        logger_msg.y_dot = rob.full_state[7]
        logger_msg.z_dot = rob.full_state[8]
        logger_msg.roll_dot = rob.full_state[9]
        logger_msg.pitch_dot = rob.full_state[10]
        logger_msg.yaw_dot = rob.full_state[11]
        logger_msg.roll_cmd = rob.full_state[12]
        logger_msg.pitch_cmd = rob.full_state[13]
        logger_msg.yaw_cmd = rob.full_state[14]
        logger_msg.thrust_cmd = rob.full_state[15]
        logger_msg.bat = rob.full_state[16]
        rob.pub.publish(msg)
        bag.write('full_quad_state', logger_msg)
        # overwrite previously published message to use vicon euler angles
        msg.roll  = rob.vicon_eulers[0]
        msg.pitch = rob.vicon_eulers[1]
        msg.yaw   = rob.vicon_eulers[2]
        #bag.write('full_quadrotor_state', logger_msg)
        r.sleep()
    bag.close()
        

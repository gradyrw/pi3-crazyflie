#!/usr/bin/env python

# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""

import sys
sys.path.append("~/crazyflie-clients-python/lib")

import cflib.crtp

import logging
import time
from threading import Timer
from threading import Thread

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie

import numpy as np

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import rospy
from pi3_crazyflie_pkg.msg import angles, rpyt, motor_msg
from std_msgs.msg import String, Float32

import numpy as np
import os.path
import datetime
import rosbag

class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """
    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        print "Connecting to %s" % link_uri
        rospy.init_node("angles_and_commander", anonymous = True) 
        self.pub = rospy.Publisher('angle_sensors', angles) 
        rospy.Subscriber("control", rpyt, self.ros_callback)
        self.roll_rate = 0
        self.pitch_rate = 0
        self.yaw_rate = 0
        self.thrust = 0
        self.kill = 0
        self.t = 0
    
        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)
        print "0"
        #Set up logging bag
        stri = String()
        #stri.data = "Alpha: " + str(rospy.get_param('ALPHA')) + " LAG: " + str(rospy.get_param('LAG')) + " VARS: " + str(rospy.get_param('VARS'))
        stri.data = 'test'
        d = datetime.datetime.now()
        path = '/home/cudauser/irobot_quad/quad_cat/logging/'
        name = str(d) + "__" + "crazyflie_angles__" + "cmds_motors"
        full_name = os.path.join(path, name + ".bag")
        self.bag = rosbag.Bag(full_name, 'w')
        self.bag.write('metadata', stri)
        print "1"
        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def ros_callback(self, cmd):
        self.roll_rate = cmd.roll_rate
        self.pitch_rate = cmd.pitch_rate
        self.yaw_rate = cmd.yaw_rate
        self.thrust = cmd.thrust
        self.kill = max(self.kill, cmd.kill)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print "Connected to %s" % link_uri
        #Start the motors
        #Thread(target=self._ramp_motors).start()
        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Sensors", period_in_ms=10)

        self._lg_stab.add_variable("stabilizer.roll", "float")
        self._lg_stab.add_variable("stabilizer.pitch", "float")
        self._lg_stab.add_variable("stabilizer.yaw", "float")
        
        self._lg_stab.add_variable("pm.vbat", "float") 
        """
        self._lg_stab.add_variable("motor.m1", "float")
        self._lg_stab.add_variable("motor.m2", "float")
        self._lg_stab.add_variable("motor.m3", "float")
        self._lg_stab.add_variable("motor.m4", "float")
        print "4"
        """
        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        self._cf.log.add_config(self._lg_stab)
        if self._lg_stab.valid:
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        else:
            print("Could not add logconfig since some variables are not in TOC")


    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print "Error when logging %s: %s" % (logconf.name, msg)

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        #print "Entered callback _stab_log_data()"

        s = angles()
        roll = data['stabilizer.roll']*np.pi/180.0
        pitch = data['stabilizer.pitch']*np.pi/180.0 
        yaw = data['stabilizer.yaw']*np.pi/180.0
        s.roll = roll
        s.pitch = pitch
        s.yaw = yaw

        s.bat = data['pm.vbat']
        
        #s.gyro_x = acceleration[0]
        #s.gyro_y = acceleration[1]
        #s.gyro_z = acceleration[2]
        self.pub.publish(s)
        self.bag.write('angles', s)
        #self.bag.write('battery', Float32(data['pm.vbat']))
        print "Printing kill" + str(self.kill)
        if (not rospy.is_shutdown() and self.kill == 0 ):
            self.t += 1
            mult = 1.0
            roll_cmd = self.roll_rate*180/np.pi * 0.5 + 2
            pitch_cmd = -(self.pitch_rate*180/np.pi) * 0.5 - 11
            yaw_cmd = -(self.yaw_rate*180/np.pi)
            thrust_cmd = ((9.81 + self.thrust) - 8.778)/2.4e-05 + 2500
            self._cf.commander.send_setpoint(roll_cmd, pitch_cmd, yaw_cmd, thrust_cmd)
            # crazy debug block
            #roll_cmd = 0
            #pitch_cmd = 30
            #yaw_cmd = 0
            #thrust_cmd = 45000
            #print "B"
            #if self.t > 2500:
            #    mult = -1.0
            #msg1 = motor_msg()
            #msg1.m1 = data['motor.m1']
            #msg1.m2 = data['motor.m2']
            #msg1.m3 = data['motor.m3']
            #msg1.m4 = data['motor.m4']
            #msg2 = rpyt()
            #msg2.roll_rate = self.roll_rate
            #msg2.pitch_rate = self.pitch_rate
            #msg2.yaw_rate = self.yaw_rate
            #msg2.thrust = self.thrust
            #msg2.kill = 0
            #self.bag.write('motors', msg1)
            #self.bag.write('commands', msg2)
            #self.pitch = mult*60
            #self.thrust = 40000
            #self._cf.commander.send_setpoint(0, self.pitch, 0, self.thrust)
        else:
            self._cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            self._cf.close_link()
            self.bag.close()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print "Disconnected from %s" % link_uri
        self.is_connected = False

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print "Scanning interfaces for Crazyflies..."
    available = cflib.crtp.scan_interfaces()
    print "Crazyflies found:"
    for i in available:
        print i[0]

    if len(available) > 0:
        le = LoggingExample(available[0][0])
    else:
        print "No Crazyflies found, cannot run example"

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)

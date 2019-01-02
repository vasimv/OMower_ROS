#!/usr/bin/env python
#
# basic localization script for locating
# the robot in rviz based on GPS data
#
# (C) Yongyang Nie, 2018
#

import rospy
import tf_conversions
import tf2_ros
import geodesy.props
import geodesy.utm
import geodesy.wu_point
import geodesy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int16MultiArray
import geometry_msgs.msg
import math
import numpy as np


class GPSLocalization():

    def __init__(self):

        rospy.init_node('omower_gps_localization')
        self.br = tf2_ros.TransformBroadcaster()
        self.sbr = tf2_ros.StaticTransformBroadcaster()
        # Subscribe to OMower's gps and imu topics
        rospy.Subscriber('/gps/coordinates', Int32MultiArray, callback=self.navsatfix_callback, queue_size=1)
        rospy.Subscriber('/imu/orientation', Int16MultiArray, callback=self.imu_callback, queue_size=1)
        # Zero UTM coordinates (set to 0)
        self.zero_x = 0
        self.zero_y = 0
        self.q_zero = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        self.q_camera = tf_conversions.transformations.quaternion_from_euler(-3.1415926/2, 0, 0)

        self.x = 0
        self.y = 0
        self.rot_x = 0
        self.rot_y = 0
        self.rot_z = 0

        self.rate = rospy.Rate(50.0)

        while not rospy.is_shutdown():
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            # Map frame (static at 0)
            t.header.frame_id = 'world'
            t.child_frame_id = 'map'
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = 0
            t.transform.rotation.x = self.q_zero[0]
            t.transform.rotation.y = self.q_zero[1]
            t.transform.rotation.z = self.q_zero[2]
            t.transform.rotation.w = self.q_zero[3]
            self.sbr.sendTransform(t)
            # Odom frame (static at 0)
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = 0
            t.transform.rotation.x = self.q_zero[0]
            t.transform.rotation.y = self.q_zero[1]
            t.transform.rotation.z = self.q_zero[2]
            t.transform.rotation.w = self.q_zero[3]
            self.sbr.sendTransform(t)
            # base_footprint frame (GPS coordinates and yaw)
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0;
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.rot_z)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.br.sendTransform(t)
            # base link frame (base_footprint with roll/pitch vector)
            t.header.frame_id = 'base_footprint'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = 0
            t.transform.translation.y = 0
            t.transform.translation.z = 0
            q = tf_conversions.transformations.quaternion_from_euler(self.rot_x, self.rot_y, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.br.sendTransform(t)
            # left camera frame (static)
            t.header.frame_id = "base_link"
            t.child_frame_id = "camera/left"
            t.transform.translation.x = -0.06
            t.transform.translation.y = 0.325
            t.transform.translation.z = 0.21
            t.transform.rotation.x = self.q_camera[0]
            t.transform.rotation.y = self.q_camera[1]
            t.transform.rotation.z = self.q_camera[2]
            t.transform.rotation.w = self.q_camera[3]
            self.sbr.sendTransform(t)
            # right camera frame (static)
            t.header.frame_id = "base_link"
            t.child_frame_id = "camera/right"
            t.transform.translation.x = 0.06
            t.transform.translation.y = 0.325
            t.transform.translation.z = 0.21
            t.transform.rotation.x = self.q_camera[0]
            t.transform.rotation.y = self.q_camera[1]
            t.transform.rotation.z = self.q_camera[2]
            t.transform.rotation.w = self.q_camera[3]
            self.sbr.sendTransform(t)
            self.rate.sleep()

        # 1. use geodesy to convert LatLong to point
        # 2. subtract the UTM static transformation from it
        # 3. send that transformation
    def imu_callback(self, msg):
        self.rot_x = (-msg.data[0] * 3.1415926) / 180.0
        self.rot_y = (msg.data[1] * 3.1415926) / 180.0
        self.rot_z = (-msg.data[2] * 3.1415926) / 180.0

    def navsatfix_callback(self, msg):

        if msg.data[2] == 0:
            print("no data received")
            
        else:
            latitude = np.float64(msg.data[0]) / np.float64(10000000.0)
            longitude = np.float64(msg.data[1]) / np.float64(10000000.0)
            point = geodesy.utm.fromLatLong(latitude, longitude).toPoint()
            # To simplify things - we take first reading as zero coordinates
            if (self.zero_x == 0):
              self.zero_x = np.float64(point.x)
              self.zero_y = np.float64(point.y)
              print "Using coordinates %f %f as zero" % (self.zero_x, self.zero_y)
            self.x = np.float64(point.x) - self.zero_x
            self.y = np.float64(point.y) - self.zero_y

if __name__ == '__main__':

    try:
        GPSLocalization()
    except rospy.ROSInterruptException:
        pass





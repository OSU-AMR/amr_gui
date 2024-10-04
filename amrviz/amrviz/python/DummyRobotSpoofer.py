#! /usr/bin/env python3

import os
import random

import rclpy
import enum
import numpy
from math import atan2
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, PoseStamped, PoseWithCovarianceStamped, TransformStamped
import rclpy.qos
import rclpy.time
from vision_msgs.msg import Detection3DArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster, TransformStamped
from transforms3d.euler import euler2quat
from transforms3d.quaternions import qmult
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from ros2node.api import get_node_names


class DummyRobotSpoofer(Node):
    def __init__(self):
        super().__init__("dummyrobotspoofer")

        self.create_timer(1.0, self.update_pose)        
        self.create_timer(1.0, self.publish_heartbeat)

        self.heartbeat_pub = self.create_publisher(Float32, "/robotdummy/heartbeat", qos_profile=rclpy.qos.qos_profile_system_default)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose = [0.0,0.0,0.0,0.0,0.0,0.0]



    def update_pose(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = "robotdummy_base"

        t.transform.translation.x = self.pose[0]
        t.transform.translation.y = self.pose[1]
        t.transform.translation.z = self.pose[2]

        q = euler2quat(self.pose[3], self.pose[4], self.pose[5])
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        self.tf_broadcaster.sendTransform(t)

        #move pose around
        self.pose = [(random.random() - .5) * .1 + self.pose[0], (random.random() - .5) * .1 + self.pose[1], 0.0, 0.0, 0.0, (random.random() - .5) * .1 + self.pose[5]]
        print(self.pose)
        
    def publish_heartbeat(self):
        msg = Float32()
        msg.data = -1.0

        self.heartbeat_pub.publish(msg)

            

        

def main(args = None):
    rclpy.init(args = args)
    rclpy.spin(DummyRobotSpoofer())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Marker Pub Keyboard-Interrupted")
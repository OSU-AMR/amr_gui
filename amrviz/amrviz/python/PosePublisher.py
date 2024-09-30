#! /usr/bin/env python3

import os

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
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from transforms3d.euler import euler2quat
from transforms3d.quaternions import qmult
from visualization_msgs.msg import Marker, MarkerArray
import yaml

class GridPublisher(Node):
    def __init__(self):
        super().__init__("gridpublisher")

        self.marker_pub = self.create_publisher(MarkerArray, "layout_array", rclpy.qos.qos_profile_system_default)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)        

        self.marker_pub_cb_timer = self.create_timer(1.0, self.MarkerPublishCallback)

        self.BroadcastGridFrame()   

        self.mesh_pkg = "amr_meshes"
        self.amrviz_pkg = "amrviz"
        self.mesh_folder = "meshes_amr"

        try:
            self.config_file = yaml.safe_load(open(os.path.join(get_package_share_directory(self.amrviz_pkg), "config", "layout_config.yaml")))
        except yaml.YAMLError as e:
            self.get_logger().info(e)

    def BroadcastGridFrame(self):

        #broadcast the grid frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "map"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0

        self.tf_static_broadcaster.sendTransform(t)


    def MarkerPublishCallback(self):
        markers = []

        tiles = self.config_file["top"]["tiles"]
        

        for tile_key in tiles.keys():
            #the yaml tile object
            tile = tiles[tile_key]


            #fill out marker header
            marker = Marker()
            marker.header.frame_id = self.config_file["top"]["map_frame"]
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = tile_key
            marker.id = 3
            marker.type = 10
            marker.action = 0
            marker.frame_locked = True

            #fill out pose fto_msg(self)or marker
            pose = Pose()
            pose.position.x = tile["pose"][0]
            pose.position.y = tile["pose"][1]
            pose.position.z = tile["pose"][2]

            #get rotation in the form of quaternion
            orientation = euler2quat(tile["pose"][3], tile["pose"][4], tile["pose"][5])

            pose.orientation.x = orientation[1]
            pose.orientation.y = orientation[2]
            pose.orientation.z = orientation[3]
            pose.orientation.w = orientation[0]

            #fill out scale for marker
            scale = Vector3()
            scale.x = tile["scale"][0]
            scale.y = tile["scale"][1]
            scale.z = tile["scale"][2]

            #color
            color = ColorRGBA()
            color.r = tile["color"][0]
            color.b = tile["color"][1]
            color.g = tile["color"][2]
            color.a = tile["color"][3]

            #duration
            duration = Duration().to_msg()
            duration.nanosec = 0
            duration.sec = 0

            marker.pose = pose
            marker.scale = scale
            marker.color = color
            marker.lifetime = duration

            #set mesh resource
            marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.mesh_pkg), self.mesh_folder, tile["mesh"], "model.ply")
            marker.mesh_use_embedded_materials = False

            markers.append(marker)
 
        msg = MarkerArray()
        msg.markers = markers

        self.marker_pub.publish(msg)

        #cancel timer so it only does this once
        self.marker_pub_cb_timer.cancel()

        print("Published the msg")

    def
        

def main(args = None):
    rclpy.init(args = args)
    rclpy.spin(GridPublisher())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Marker Pub Keyboard-Interrupted")
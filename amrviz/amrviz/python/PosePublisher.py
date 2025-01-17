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
from transforms3d.euler import euler2quat, quat2rotm
from transforms3d.quaternions import qmult
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from ros2node.api import get_node_names


class GridPublisher(Node):
    def __init__(self):
        super().__init__("gridpublisher")

        #publishers
        self.layout_pub = self.create_publisher(MarkerArray, "layout_array", rclpy.qos.qos_profile_system_default)
        self.robot_pub = self.create_publisher(MarkerArray, "robot_array", rclpy.qos.qos_profile_system_default)
        
        #timers
        self.marker_pub_cb_timer = self.create_timer(1.0, self.MarkerPublishCallback)
        self.robot_pub_timer = self.create_timer(1.0, self.updateRobotStatus)

        #tf
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)        

        self.BroadcastGridFrame()   

        self.mesh_pkg = "amr_meshes"
        self.amrviz_pkg = "amrviz"
        self.mesh_folder = "meshes_amr"


        #load in yaml
        try:
            self.config_file = yaml.safe_load(open(os.path.join(get_package_share_directory(self.amrviz_pkg), "config", "layout_config.yaml")))
        except yaml.YAMLError as e:
            self.get_logger().info(e)

        #create robot dict
        self.robot_status = dict()

        names = self.config_file["top"]["robots"].keys()
        for name in names:
            self.robot_status[name] = 0
        


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

        self.layout_pub.publish(msg)

        #cancel timer so it only does this once
        self.marker_pub_cb_timer.cancel()

        print("Published the msg")

    def updateRobotStatus(self):
        current_time = self.get_clock().now().nanoseconds


        #check for heartbeat topics
        for node_name in get_node_names(node=self):

            print(str(node_name))
            try:
                for topic in self.get_publisher_names_and_types_by_node(node_name.name, ""):

                    #if the topic is a heartbeat topic and is a valid robot
                    if "heartbeat" in topic[1] and node_name in self.robot_status.keys():

                        #reset the robot timer
                        self.robot_status[node_name] = current_time
            except:
                pass
        
        markers = []

        robots = self.config_file["top"]["robots"]

        

        for robot_key in robots.keys():
            #the yaml tile object
            robot = robots[robot_key]

            #fill out marker header
            marker = Marker()
            marker.header.frame_id = robot["frame"]
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = robot_key
            marker.id = 3
            marker.type = 10
            marker.action = 0
            marker.frame_locked = True

            #fill out pose fto_msg(self)or marker
            pose = Pose()
            pose.position.x = robot["pose"][0]
            pose.position.y = robot["pose"][1]
            pose.position.z = robot["pose"][2]

            #get rotation in the form of quaternion
            orientation = euler2quat(robot["pose"][3], robot["pose"][4], robot["pose"][5])

            pose.orientation.x = orientation[1]
            pose.orientation.y = orientation[2]
            pose.orientation.z = orientation[3]
            pose.orientation.w = orientation[0]

            #fill out scale for marker
            scale = Vector3()
            scale.x = robot["scale"][0]
            scale.y = robot["scale"][1]
            scale.z = robot["scale"][2]

            #color
            color = ColorRGBA()
            color.r = robot["color"][0]
            color.b = robot["color"][1]
            color.g = robot["color"][2]

            if (current_time - self.robot_status[robot_key]) > 5e9:
                color.a = robot["color"][3]
            else:
                color.a = 0

            #duration
            duration = Duration().to_msg()
            duration.nanosec = 0
            duration.sec = 0

            marker.pose = pose
            marker.scale = scale
            marker.color = color
            marker.lifetime = duration

            #set mesh resource
            marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.mesh_pkg), self.mesh_folder, robot["mesh"], "model.dae")
            marker.mesh_use_embedded_materials = False

            markers.append(marker)
 
        msg = MarkerArray()
        msg.markers = markers

        self.robot_pub.publish(msg)

        #cancel timer so it only does this once
        self.marker_pub_cb_timer.cancel()

        print("Published the msg")

            
    def configure_tile(self, key, config):

        #list of markers to be added
        marker_list = []
        
        #fill out marker header
        marker = Marker()
        marker.header.frame_id = self.config_file["top"]["map_frame"]
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = key
        marker.id = 3
        marker.type = 10
        marker.action = 0
        marker.frame_locked = True

        #fill out pose fto_msg(self)or marker
        pose = Pose()
        pose.position.x = config["pose"][0]
        pose.position.y = config["pose"][1]
        pose.position.z = config["pose"][2]

        #get rotation in the form of quaternion
        orientation = euler2quat(config["pose"][3], config["pose"][4], config["pose"][5])

        pose.orientation.x = orientation[1]
        pose.orientation.y = orientation[2]
        pose.orientation.z = orientation[3]
        pose.orientation.w = orientation[0]

        #fill out scale for marker
        scale = Vector3()
        scale.x = config["scale"][0]
        scale.y = config["scale"][1]
        scale.z = config["scale"][2]

        #color
        color = ColorRGBA()
        color.r = config["color"][0]
        color.b = config["color"][1]
        color.g = config["color"][2]
        color.a = config["color"][3]

        #duration
        duration = Duration().to_msg()
        duration.nanosec = 0
        duration.sec = 0

        marker.pose = pose
        marker.scale = scale
        marker.color = color
        marker.lifetime = duration

        mesh = "tile"

        #handle mesh cases
        match config["mesh"]:

            case "tile_T":
                #tile mesh
                mesh = "tile"

                #cook up tape
                
                


            case "tile":                
                mesh = "tile"

            case _:
                #default to plain tile
                mesh = "tile"
                print(f"Unrecognized marker name {config["mesh"]}")


        #set mesh resource
        marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.mesh_pkg), self.mesh_folder, mesh, "model.ply")
        marker.mesh_use_embedded_materials = False
        
        marker_list.append(marker)

        return marker_list
    
    def configure_tape_strip(self, tile_config, tape_config):
        #fill out marker header
        marker = Marker()
        marker.header.frame_id = self.config_file["top"]["map_frame"]
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = key
        marker.id = 3
        marker.type = 10
        marker.action = 0
        marker.frame_locked = True

        #tile orientation
        tile_orientation = euler2quat(tile_config["pose"][3], tile_config["pose"][4], tile_config["pose"][5])

        #tile_rotm
        tile_rotm = quat2rotm(tile_orientation)


        #fill out pose fto_msg(self)or marker
        pose = Pose()
        pose.position.x = tape_config["pose"][0] + tile_config[["pose"][0]]
        pose.position.y = tape_config["pose"][1] + tile_config[["pose"][0]]
        pose.position.z = tape_config["pose"][2] + tile_config[["pose"][0]]

        #get rotation in the form of quaternion
        orientation = euler2quat(tape_config["pose"][3], tape_config["pose"][4], tape_config["pose"][5])

        pose.orientation.x = orientation[1]
        pose.orientation.y = orientation[2]
        pose.orientation.z = orientation[3]
        pose.orientation.w = orientation[0]

        #fill out scale for marker
        scale = Vector3()
        scale.x = config["scale"][0]
        scale.y = config["scale"][1]
        scale.z = config["scale"][2]

        #color
        color = ColorRGBA()
        color.r = tape_config["color"][0]
        color.b = tape_config["color"][1]
        color.g = tape_config["color"][2]
        color.a = tape_config["color"][3]

        #duration
        duration = Duration().to_msg()
        duration.nanosec = 0
        duration.sec = 0

        marker.pose = pose
        marker.scale = scale
        marker.color = color
        marker.lifetime = duration
                
        #set mesh resource
        marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.mesh_pkg), self.mesh_folder, tape_config["mesh"], "model.ply")
        marker.mesh_use_embedded_materials = False

        return marker

        
        

def main(args = None):
    rclpy.init(args = args)
    rclpy.spin(GridPublisher())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Marker Pub Keyboard-Interrupted")
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
from std_msgs.msg import ColorRGBA, Int16MultiArray
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from transforms3d.euler import euler2quat, quat2mat
from transforms3d.quaternions import qmult
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from ros2node.api import get_node_names

TAPE_DEFAULT_LENGTH = .3038
ROUTE_STATUS_TAPE_HEIGHT = .006
ROUTE_BLOCKED_COLOR = [1.0, 0.2, 0.2, 0.6]

class GridPublisher(Node):

    edge_file = None

    def __init__(self):
        super().__init__("gridpublisher")

        #publishers
        self.layout_pub = self.create_publisher(MarkerArray, "layout_array", rclpy.qos.qos_profile_system_default)
        self.closed_path_pub = self.create_publisher(MarkerArray, "closed_paths", rclpy.qos.qos_profile_system_default)
        self.robot_pub = self.create_publisher(MarkerArray, "robot_array", rclpy.qos.qos_profile_system_default)

        #subscriber
        self.blocked_edge_subs = self.create_subscription(Int16MultiArray, "blocked_edges", self.update_blocked_routes, qos_profile=rclpy.qos.qos_profile_system_default)
        
        #timers
        self.marker_pub_cb_timer = self.create_timer(2.0, self.MarkerPublishCallback)
        self.robot_pub_timer = self.create_timer(1.0, self.updateRobotStatus)

        #tf
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)        

        self.BroadcastGridFrame()   

        #previously blocked paths
        self.previously_blocked_paths = []

        self.mesh_pkg = "amr_meshes"
        self.amrviz_pkg = "amrviz"
        self.mesh_folder = "meshes_amr"
        self.central_pkg = "amr_central"

        #load in yaml
        try:
            self.config_file = yaml.safe_load(open(os.path.join(get_package_share_directory(self.amrviz_pkg), "config", "layout_config.yaml")))
        except yaml.YAMLError as e:
            self.get_logger().info(e)

        try:
            self.tape_mesh = self.config_file["top"]["tape_configurations"]["T"]["strip1"]["mesh"]
            self.first_tile = self.config_file["top"]["tiles"]["tile0"]
        except:
            self.get_logger().warn("Cannot load any tiles, this may cause other features not to function correctly!")
            self.tape_mesh = None
            self.first_tile = None

        #load in edge yaml
        try:
            self.edge_file = yaml.safe_load(open(os.path.join(get_package_share_directory(self.central_pkg), "test_data", "edge_positions.yaml")))
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

            spur_configurations = dict()
            try:
                spur_configurations = tile["spurs"]
            except:
                #is expected
                pass

            tile_markers = self.configure_tile(tile_key, tile, self.config_file["top"]["tape_configurations"], spur_configurations)

            for marker in tile_markers:
                markers.append(marker)
 
        tags = self.config_file["top"]["tags"]

        for tag_key in tags.keys():
            #the yaml tile object
            tag = tags[tag_key]

            tag_markers = self.configure_tile(tag_key, tag, self.config_file["top"]["tape_configurations"], dict()) #cursed

            for marker in tag_markers:
                markers.append(marker)

        
        msg = MarkerArray()

        #helper objects
        helpers = self.config_file["top"]["helpers"]
        for helper_key in helpers:
            helper_config = self.config_file["top"]["helpers"][helper_key]
            marker = self.configure_helpers(helper_config, helper_key)
            markers.append(marker)

        msg.markers = markers

        self.layout_pub.publish(msg)

        #cancel timer so it only does this once
        self.marker_pub_cb_timer.cancel()

        print("Published the msg")

    def configure_helpers(self, config, key):
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

        mesh = config["mesh"]
        marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.mesh_pkg), self.mesh_folder, mesh, "model.dae")
        marker.mesh_use_embedded_materials = False
            
        return marker

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

        print("Published the msg")

    def update_blocked_routes(self, msg):
        #update

        if(self.edge_file == None or self.tape_mesh == None or self.first_tile == None):
            return

        markers = []

        edges_for_removal = self.previously_blocked_paths

        for edge in msg.data:

            if(edge in edges_for_removal):
                edges_for_removal.remove(edge)

            #edge string
            str = f"edge_{edge}"

            edge_data = self.edge_file[str]

            marker = Marker()
            marker.header.frame_id = self.config_file["top"]["map_frame"]
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = str
            marker.id = 3
            marker.type = 10
            marker.action = 0
            marker.frame_locked = True

            #fill out pose fto_msg(self)or marker
            pos_x = (edge_data["start_x_disp"] + edge_data["end_x_disp"]) / 2
            pos_y = (edge_data["start_y_disp"] + edge_data["end_y_disp"]) / 2

            #center of tape

            pose = Pose()
            pose.position.x = pos_x
            pose.position.y = pos_y
            pose.position.z = ROUTE_STATUS_TAPE_HEIGHT

            #get rotation in the form of quaternion
            angle = atan2((edge_data["end_y_disp"] - edge_data["start_y_disp"]), (edge_data["end_x_disp"] - edge_data["start_x_disp"]))

            orientation = euler2quat(0.0, 0.0, angle)

            pose.orientation.x = orientation[1]
            pose.orientation.y = orientation[2]
            pose.orientation.z = orientation[3]
            pose.orientation.w = orientation[0]

            #fill out scale for marker
            #get path length
            path_length = pow(((edge_data["end_y_disp"] - edge_data["start_y_disp"])**2 + (edge_data["end_x_disp"] - edge_data["start_x_disp"])**2), .5)
            scale_num = path_length / TAPE_DEFAULT_LENGTH

            scale = Vector3()
            scale.x = scale_num * self.first_tile["scale"][0]
            scale.y = self.first_tile["scale"][1]
            scale.z = self.first_tile["scale"][2]

            #color
            color = ColorRGBA()
            color.r = ROUTE_BLOCKED_COLOR[0]
            color.b = ROUTE_BLOCKED_COLOR[1]
            color.g = ROUTE_BLOCKED_COLOR[2]
            color.a = ROUTE_BLOCKED_COLOR[3]

            #duration
            duration = Duration().to_msg()
            duration.nanosec = 0
            duration.sec = 0

            marker.pose = pose
            marker.scale = scale
            marker.color = color
            marker.lifetime = duration
                    
            #set mesh resource
            marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.mesh_pkg), self.mesh_folder, self.tape_mesh, "model.ply")
            marker.mesh_use_embedded_materials = False

            markers.append(marker)

        #mark the edges for removal
        for edge in edges_for_removal:

            #edge string
            str = f"edge_{edge}"

            edge_data = self.edge_file[str]

            marker = Marker()
            marker.header.frame_id = self.config_file["top"]["map_frame"]
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = str
            marker.id = 3
            marker.type = 10
            marker.action = 2
            marker.frame_locked = True
            
            markers.append(marker)


        self.previously_blocked_paths = msg.data

        msg = MarkerArray()
        msg.markers = markers

        self.closed_path_pub.publish(msg)

    def configure_tile(self, key, config, tape_configs, spur_configs):

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
                T_config = tape_configs["T"]
                for tape_key in T_config.keys():
                    strip = self.configure_tape_strip(key + "/" + tape_key, config, T_config[tape_key])
                    marker_list.append(strip)

            case "tile_X":
                #tile mesh
                mesh = "tile"

                #cook up tape
                T_config = tape_configs["X"]
                for tape_key in T_config.keys():
                    strip = self.configure_tape_strip(key + "/" + tape_key, config, T_config[tape_key])
                    marker_list.append(strip)

            case "tile_l":
                #tile mesh
                mesh = "tile"

                #cook up tape
                T_config = tape_configs["l"]
                for tape_key in T_config.keys():
                    strip = self.configure_tape_strip(key + "/" + tape_key, config, T_config[tape_key])
                    marker_list.append(strip)

            case "tile_r":
                #tile mesh
                mesh = "tile"

                #cook up tape
                T_config = tape_configs["r"]
                for tape_key in T_config.keys():
                    strip = self.configure_tape_strip(key + "/" + tape_key, config, T_config[tape_key])
                    marker_list.append(strip)

            case "tile":                
                mesh = "tile"

            case _:
                #default to plain tile
                mesh = "tile"
                attempt = config["mesh"]
                print(f"Unrecognized marker name {attempt}")


        #set mesh resource
        marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.mesh_pkg), self.mesh_folder, mesh, "model.ply")
        marker.mesh_use_embedded_materials = False

        marker_list.append(marker)
        
        for index, spur in enumerate(spur_configs.keys()):

            #configure the spur tape strip

            spur_config = spur_configs[spur]
            #add the spur color in
            spur_config.update(tape_configs["spur_supplimental"])
            marker_list.append(self.configure_tape_strip(key + "/spur_" + str(index), config, spur_config))
            
        return marker_list

    def configure_tape_strip(self, key, tile_config, tape_config):
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
        tile_rotm = quat2mat(tile_orientation)
        
        #rotated tile_pose
        adjusted_pose = numpy.matmul(tile_rotm, [tape_config["pose"][0], tape_config["pose"][1], tape_config["pose"][2]])

        #fill out pose fto_msg(self)or marker
        pose = Pose()
        pose.position.x = tile_config["pose"][0] + adjusted_pose[0]
        pose.position.y = tile_config["pose"][1] + adjusted_pose[1]
        pose.position.z = tile_config["pose"][2] + adjusted_pose[2]

        #get rotation in the form of quaternion
        orientation = euler2quat(tape_config["pose"][3], tape_config["pose"][4], tape_config["pose"][5])
        orientation = qmult(tile_orientation, orientation)

        pose.orientation.x = orientation[1]
        pose.orientation.y = orientation[2]
        pose.orientation.z = orientation[3]
        pose.orientation.w = orientation[0]

        #fill out scale for marker
        scale = Vector3()
        scale.x = tile_config["scale"][0] * tape_config["scale"][0]
        scale.y = tile_config["scale"][1] * tape_config["scale"][1]
        scale.z = tile_config["scale"][2] * tape_config["scale"][2]

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
    
    def configure_destination(self, key, config, color):
        marker = Marker()
        marker.header.frame_id = self.config_file["top"]["map_frame"]
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = key
        marker.id = 3
        marker.type = 2
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
        color.r = color["color"][0]
        color.b = color["color"][1]
        color.g = color["color"][2]
        color.a = color["color"][3]

        #duration
        duration = Duration().to_msg()
        duration.nanosec = 0
        duration.sec = 0

        marker.pose = pose
        marker.scale = scale
        marker.color = color
        marker.lifetime = duration

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
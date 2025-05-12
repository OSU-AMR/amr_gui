#! /usr/bin/env python3

import random

import rclpy
from math import atan2
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import TransformStamped
import rclpy.qos
import rclpy.time
from rclpy.node import Node
from amr_msgs.msg import Heartbeat
from tf2_ros import TransformBroadcaster, TransformStamped
from transforms3d.euler import euler2quat
from time import sleep
from math import floor
from std_msgs.msg import Float32, String



class DummyRobotSpoofer(Node):
    rfid_pub_count = 0

    def __init__(self):
        
        super().__init__(f"dummyrobotspoofer_{floor(random.random() * 100000)}")

        self.create_timer(1.0, self.update_pose)        
        self.create_timer(1.0, self.publish_heartbeat)

        self.declare_parameter("robot_name", "robotdummy")
        self.declare_parameter("voltage_level", -1.0)
        self.declare_parameter("encoder_pub_interval", 0.5)
        self.declare_parameter("ir_pub_interval", 0.5)
        self.declare_parameter("rfid_pub_interval", 2.5)

        self.heartbeat_pub = self.create_publisher(Heartbeat, "/" + str(self.get_parameter("robot_name").value) + "/heartbeat", qos_profile=rclpy.qos.qos_profile_system_default)
        self.encoder_l_pub = self.create_publisher(Float32, "/" + str(self.get_parameter("robot_name").value) + "/state/encoder/left", qos_profile=rclpy.qos.qos_profile_system_default)
        self.encoder_r_pub = self.create_publisher(Float32, "/" + str(self.get_parameter("robot_name").value) + "/state/encoder/right", qos_profile=rclpy.qos.qos_profile_system_default)
        self.ir_l_pub = self.create_publisher(Float32, "/" + str(self.get_parameter("robot_name").value) + "/state/ir/left", qos_profile=rclpy.qos.qos_profile_system_default)
        self.ir_r_pub = self.create_publisher(Float32, "/" + str(self.get_parameter("robot_name").value) + "/state/ir/right", qos_profile=rclpy.qos.qos_profile_system_default)
        self.rfid_pub = self.create_publisher(String, "/" + str(self.get_parameter("robot_name").value) + "/state/rfid", qos_profile=rclpy.qos.qos_profile_system_default)

        self.create_timer(self.get_parameter("encoder_pub_interval").value, self.publish_encoders)
        self.create_timer(self.get_parameter("ir_pub_interval").value, self.publish_ir)
        self.create_timer(self.get_parameter("rfid_pub_interval").value, self.publish_rfid)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose = [0.0,0.0,0.0,0.0,0.0,0.0]

        self.name_cb_timer = self.create_timer(1.0, self.set_name_cb)
        

    def set_name_cb(self):
        if(str(self.get_parameter("robot_name").value ) != "robotdummy"):
            self.heartbeat_pub = self.create_publisher(Heartbeat, "/" + str(self.get_parameter("robot_name").value) + "/heartbeat", qos_profile=rclpy.qos.qos_profile_system_default)

            self.name_cb_timer.cancel()

        

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
        # self.pose = [(random.random() - .5) * .1 + self.pose[0], (random.random() - .5) * .1 + self.pose[1], 0.0, 0.0, 0.0, (random.random() - .5) * .1 + self.pose[5]]
        # print(self.pose)
        


    
    def publish_heartbeat(self):
        msg = Heartbeat()
        msg.robot_name = "robotdummy"
        msg.battery_voltage = self.get_parameter("voltage_level").value
        msg.uid_connected = False
        msg.left_ir_connected = False
        msg.right_ir_connected = False
        msg.left_esc_connected = False
        msg.right_esc_connected = False

        self.heartbeat_pub.publish(msg)

    def publish_encoders(self):
        #pulbish out fake encoder data
        #left will always be the opposite of right

        msg = Float32()
        msg.data = random.random() * 10000 - 5000
        self.encoder_l_pub.publish(msg)

        msg.data = msg.data * -1
        self.encoder_r_pub.publish(msg)

    def publish_ir(self):
        #pulbish out fake ir data
        #left will always be the opposite of right

        msg = Float32()
        msg.data = random.random() * 2000 - 1000
        self.ir_l_pub.publish(msg)

        msg.data = msg.data * -1
        self.ir_r_pub.publish(msg)

    def publish_rfid(self):
        #publish ir sensor
        msg = String()
        msg.data = "ABCD" + str(self.rfid_pub_count)
        self.rfid_pub.publish(msg)

        self.rfid_pub_count += 1
            

def main(args = None):
    rclpy.init(args = args)
    rclpy.spin(DummyRobotSpoofer())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Marker Pub Keyboard-Interrupted")
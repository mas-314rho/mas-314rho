import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_system_default
import random

import sys

def main(args=None):

    rclpy.init(args=sys.argv)

    node=rclpy.create_node('init_pose')

    publisher= node.create_publisher(PoseWithCovarianceStamped,'/initialpose',qos_profile_system_default) 
    rdnposex = random.uniform(-2.,2.) #selecting a random position on the x axis
    rdnposey = random.uniform(-2.,2.) #selecting a random position on the y axis
    rdnorientz = random.uniform(0,360)    #selecting a random orientation around z axis

    initpose_msg = PoseWithCovarianceStamped()
    initpose_msg.header.frame_id = "map"
    initpose_msg.pose.pose.position.x = rdnposex
    initpose_msg.pose.pose.position.y = rdnposey
    initpose_msg.pose.pose.orientation.x = 0.0
    initpose_msg.pose.pose.orientation.y = 0.0
    initpose_msg.pose.pose.orientation.z = rdnorientz
    initpose_msg.pose.pose.orientation.w = 0.0

    #node.sleep(1)

    node.get_logger().info( "Setting initial pose")
    publisher.publish(initpose_msg)
    node.get_logger().info( "Initial pose SET")

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

############################################################################################
#

#
############################################################################################

import os
import cv2
import time
import math
import numpy as np
import pyrealsense2 as rs
from pathlib import Path
import sys
import tf2_ros
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
import math

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped



################################# CLASS INITIALIZATION #####################################

np.set_printoptions(threshold=sys.maxsize)


class Publisher(Node):

    # Initialization of the class
    def __init__(self):
        super().__init__('cloud_publisher')

        # Initialize logs level
        try:
            log_level = int(os.environ['LOG_LEVEL'])
        except:
            log_level = 20
            self.get_logger().info("LOG_LEVEL not defined, setting default: INFO")

        # Initialize RealSense D435 camera
        config = rs.config()
        self.pipe = rs.pipeline()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #self.decimate = rs.decimation_filter(1)
        # decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)

        d435_attempt = False
        while d435_attempt == False:
            try:
                self.pipe.start(config)
                self.get_logger().info("Camera Found!")
                d435_attempt = True
            except RuntimeError:
                self.get_logger().error("Device not connected. Connect your Intel RealSense camera")

        # Initialize variables
        qos = QoSProfile(depth=10)
        pointcloud_topic = '/camera/cloud'
        print(f"Publish pointcloud on topic {pointcloud_topic}")
        self.point_cloud = rs.pointcloud()
        self.mean_time = 0.0
        self.n_data = 0
        #node = rclpy.create_node('robot_depth_tf_broadcaster')
        self.br = tf2_ros.StaticTransformBroadcaster(self)

        # Create publisher
        self.pc_pub = self.create_publisher(
            PointCloud2,
            pointcloud_topic,
            qos)
        self.pc_pub_rot = self.create_publisher(
            PointCloud2,
            "/camera/cloud_rot",
            qos)

        self.get_logger().info("Initializing process")
        self.theta = 3.1416/6
        self.xTransl =  -0.5

        self.RotTranslMat = np.matrix([[math.cos(self.theta), -math.sin(self.theta), 0, 0],
                                 [math.sin(self.theta), math.cos(self.theta), 0, 0],
                                 [0, 0, 1, self.xTransl],
                                 [0, 0, 0, 1]])

        self.RotMat = np.matrix([[1.0, .0, .0], [.0, math.cos(self.theta), -math.sin(self.theta)],
                                   [.0, math.sin(self.theta), math.cos(self.theta)]
                                   ])
        self.process()

    #################################### MAIN METHODS ##########################################
    
    def process(self):
        while True:
             self.depth_method()

    def depth_method(self):
        # Get the depth frame and publish the pointcloud and the reference frame transform
        frames = self.pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        begin = time.time()
        depth_frame = self.decimate.process(depth_frame)
        points = self.point_cloud.calculate(depth_frame)
        v = points.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)
        verts_trans = np.transpose(verts.copy())
        verts_trans[1, :] += self.xTransl
        verts_rot = np.transpose(np.dot(self.RotMat, verts_trans)) 
        pc_msg = self.point_cloud_message(verts, "depth_frame")
        pc_msg_rot = self.point_cloud_message(verts_rot, "depth_frame")
        self.pc_pub.publish(pc_msg)
        self.pc_pub_rot.publish(pc_msg_rot)
        self.br.sendTransform(self.tf_message(pc_msg.header.stamp))
        end = time.time()
        if self.mean_time == 0.0:
                self.mean_time = end - begin
                self.n_data = 1
        else:
                self.mean_time = (self.mean_time * self.n_data + end - begin) / (self.n_data + 1)
                self.n_data += 1
        if self.n_data % 20 == 0:
                self.get_logger().debug(f"Mean computing time: {self.mean_time:.3f}")
                

    def tf_message(self, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        quat = self.quaternion_from_euler(-1.5708, 0, -1.5708)
        tf_msg.child_frame_id = "depth_frame"
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.10
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        return tf_msg

    def point_cloud_message(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        From: https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_publisher/pcd_publisher_node.py
        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        References:
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
            http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
        """
        # In a PointCloud2 message, the point cloud is stored as an byte
        # array. In order to unpack it, we also include some parameters
        # which desribes the size of each individual point.

        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes()

        # The fields specify what the bytes represents. The first 4 bytes
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [PointField(
                name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
                for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which
        # coordinate frame it is represented in.
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=parent_frame)

        return PointCloud2(
                header=header,
                height=1,
                width=points.shape[0],
                is_dense=True,
                is_bigendian=False,
                fields=fields,
                point_step=(itemsize * 3),  # Every point consists of three float32s.
                row_step=(itemsize * 3 * points.shape[0]),
                data=data
        )
       
    @staticmethod
    def quaternion_from_euler(yaw, pitch, roll):
        #From http://docs.ros.org/en/jade/api/tf2/html/Quaternion_8h_source.html
        half_yaw = yaw * 0.5
        half_pitch = pitch * 0.5
        half_roll = roll * 0.5 
        cos_yaw = math.cos(half_yaw);
        sin_yaw = math.sin(half_yaw);
        cos_pitch = math.cos(half_pitch);
        sin_pitch = math.sin(half_pitch);
        cos_roll = math.cos(half_roll);
        sin_roll = math.sin(half_roll);
        quat = (sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw, #x
                cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw, #y
                cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw, #z
                cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw) #w
        norm = math.sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2]
            + quat[3] * quat[3])
        norm_quat = [entry / norm for entry in quat]
        return norm_quat
                
################################# MAIN #############################################


def main(args=None):
    rclpy.init()
    publisher = Publisher()

    try:
            rclpy.spin(publisher)
    except KeyboardInterrupt:
            publisher.get_logger().info("Shutting down")
    publisher.pipe.stop()
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
        main()


        tf_msg.header.frame_id = "base_link"

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

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped



################################# CLASS INITIALIZATION #####################################

np.set_printoptions(threshold=sys.maxsize)


class RealsenseCamera:
    '''
    Abstraction of any RealsenseCamera. From https://github.com/ivomarvan/
    samples_and_experiments/blob/master/Multiple_realsense_cameras/multiple_realsense_cameras.py
    '''
    _colorizer = rs.colorizer()

    def __init__(
        self,
        serial_number :str,
        name: str,
        point_cloud_only: bool
    ):
        self._serial_number = serial_number
        self._name = name
        self._pipeline = None
        self._started = False
        self._pc_only = point_cloud_only
        
        self.__start_pipeline()



    def __del__(self):
        if self._started and not self._pipeline is None:
            self._pipeline.stop()
            
            
    def get_full_name(self):
        return f'{self._name} ({self._serial_number})'


    def __start_pipeline(self):
        # Configure depth and color streams
        self._pipeline = rs.pipeline()
        config = rs.config()
        if self._pc_only:
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            #self.decimate = rs.decimation_filter(1)
        config.enable_device(self._serial_number)
        start_attempt = False
        while start_attempt == False:
            try: 
                self._pipeline.start(config)
                start_attempt = True
            except RuntimeError:
                print("Device not connected. Connect your Intel RealSense camera")
        self._started = True
        print(f'{self.get_full_name()} camera is ready.')


    def get_frames(self) -> [rs.frame]:
        '''
        Return a frame do not care about type
        '''
        frameset = self._pipeline.wait_for_frames()
        if frameset:
            return [f for f in frameset]
        else:
            return []

    def get_frameset(self) -> rs.frame:
        return self._pipeline.wait_for_frames()
    
    def get_depth_frame(self) -> rs.depth_frame: 
        frameset = self._pipeline.wait_for_frames()
        return frameset.get_depth_frame()

    @classmethod
    def get_title(cls, frame: rs.frame, whole: bool) -> str:
        # <pyrealsense2.video_stream_profile: Fisheye(2) 848x800 @ 30fps Y8>
        profile_str = str(frame.profile)
        first_space_pos = profile_str.find(' ')
        whole_title = profile_str[first_space_pos + 1: -1]
        if whole:
            return whole_title
        return whole_title.split(' ')[0]


    @classmethod
    def get_images_from_video_frames(cls, frames: [rs.frame]) -> ([(np.ndarray, rs.frame)] , [rs.frame], int, int):
        '''
        From all the frames, it selects those that can be easily interpreted as pictures.
        Converts them to images and finds the maximum width and maximum height from all of them.
        '''
        max_width = -1
        max_height = -1
        img_frame_tuples = []
        unused_frames = []
        for frame in frames:
            if frame.is_video_frame():
                if frame.is_depth_frame():
                    img = np.asanyarray(RealsenseCamera._colorizer.process(frame).get_data())
                else:
                    img = np.asanyarray(frame.get_data())
                    img = img[...,::-1].copy()  # RGB<->BGR
                max_height = max(max_height, img.shape[0])
                max_width  = max(max_width, img.shape[1])
                img_frame_tuples.append((img,frame))
            else:
                unused_frames.append(frame)
        return img_frame_tuples, unused_frames, max_width, max_height




class AllCamerasLoop:
    '''
    Take info from all connected cameras in the loop. From https://github.com/ivomarvan/
    samples_and_experiments/blob/master/Multiple_realsense_cameras/multiple_realsense_cameras.py
    '''

    def __init__(self, point_cloud_only: False):
        self._cameras = self.get_all_connected_cameras(point_cloud_only)

    def get_frames(self) -> [rs.frame]:
        '''
        Return frames in given order. 
        '''
        ret_frames = []

        for camera in self._cameras:
            frames = camera.get_frames()
            if frames:
                ret_frames += frames
        return ret_frames
    
    
    def get_depth_frames(self) -> [rs.depth_frame]:
        ret_frames = []

        for camera in self._cameras:
            frame = camera.get_depth_frame()
            if frame:
                ret_frames.append(frame)
        return ret_frames

    def __get_window_name(self):
        s = ''
        for camera in self._cameras:
            if s:
                s += ', '
            s += camera.get_full_name()
        return s
    
     
    def __get_connected_cameras_info(self, camera_name_suffix: str = 'T265') -> [(str, str)]:
        '''
        Return list of (serial number,names) conected devices.
        Eventualy only fit given suffix (like T265, D415, ...)
        (based on https://github.com/IntelRealSense/librealsense/issues/2332)
        '''
        ret_list = []
        ctx = rs.context()
        for d in ctx.devices:
            serial_number = d.get_info(rs.camera_info.serial_number)
            name = d.get_info(rs.camera_info.name)
            if camera_name_suffix and not name.endswith(camera_name_suffix):
                continue
            ret_list.append((serial_number, name))
        return ret_list


    def get_all_connected_cameras(self, point_cloud_only: bool) -> [RealsenseCamera]:
        cameras = self.__get_connected_cameras_info(camera_name_suffix=None)
        return [RealsenseCamera(serial_number, name, point_cloud_only) for serial_number, name in cameras]

    def run_loop(self):
        stop = False
        window = ImgWindow(name=self.__get_window_name())
        while not stop:
            frames = self.get_frames()
            window.swow(self.__frames_interpreter.get_image_from_frames(frames))
            stop = window.is_stopped()



class Publisher(Node):
    time_it = False
    # Initialization of the class
    def __init__(self, multiple_cameras=False):
        super().__init__('cloud_publisher')

        # Initialize logs level
        try:
            log_level = int(os.environ['LOG_LEVEL'])
        except:
            log_level = 20
            self.get_logger().info("LOG_LEVEL not defined, setting default: INFO")

        # Initialize RealSense cameras
        self.camera_loop = AllCamerasLoop(point_cloud_only=True)


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
        depth_frames = self.camera_loop.get_depth_frames()
        #depth_frame = frames.get_depth_frame()
        if time_it:
            begin = time.time()
        #depth_frame = self.decimate.process(depth_frame)
        points = self.point_cloud.calculate(depth_frames[0])
        v = points.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)
        verts_trans = np.transpose(verts.copy())
        verts_trans[1, :] += self.xTransl
        verts_rot = np.transpose(np.dot(self.RotMat, verts_trans)) 
        pc_msg = self.point_cloud_message(verts, "depth_frame")
        pc_msg_rot = self.point_cloud_message(verts_rot, "depth_frame")
        self.pc_pub.publish(pc_msg)
        self.pc_pub_rot.publish(pc_msg_rot)
        self.publish_tf_transform
        if time_it:
            end = time.time()
            if self.mean_time == 0.0:
                    self.mean_time = end - begin
                    self.n_data = 1
            else:
                    self.mean_time = (self.mean_time * self.n_data + end - begin) / (self.n_data + 1)
                    self.n_data += 1
            if self.n_data % 20 == 0:
                    self.get_logger().debug(f"Mean computing time: {self.mean_time:.3f}")
                    
    
    def publish_tf_transform(self):
        self.br.sendTransform(self.tf_message_camera0(pc_msg.header.stamp))


    def tf_message_camera0(self, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "base_link"
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
    
    def get_camera_extrinsics(self):
    ''' Method that gets the camera extrinsics for each camera serial number, from a yaml file
    '''
        return
                
################################# MAIN #############################################


def main(multiple_cameras=False):
    rclpy.init()
    publisher = Publisher(multiple_cameras=False)

    try:
            rclpy.spin(publisher)
    except KeyboardInterrupt:
            publisher.get_logger().info("Shutting down")
    publisher.pipe.stop()
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
        main()

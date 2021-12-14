# pyrealsense2_pointcloud
Ros2 package that uses pyrealsense2 to get the pointcloud and publishes it as a ros2 message.  
It can also publish a tf_message between a reference frame and the pointcloud reference frame. Otherwise it can be set in the cloud.launch.py file, via the static_transform_publisher node.

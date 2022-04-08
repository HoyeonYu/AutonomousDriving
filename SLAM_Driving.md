# SLAM Driving

1. Create Mapping Luanch File
	```xml
	<launch>
	  <param name="robot_description" textfile="$(find xycar_slam)/urdf/xycar.urdf" />
	  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

	  <node name="cartographer_node" pkg="cartographer_ros" type="cartographer_node" args="        
		-configuration_directory $(find xycar_slam)/config        
		-configuration_basename mapping.lua" />  
	  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros" type="cartographer_occupancy_grid_node" args="
		-resolution 0.05" />

	  <node name="rviz" pkg="rviz" type="rviz" args=" -d $(find xycar_slam)/rviz/mapping.rviz" />

	  <!-- Record ROS Bag File -->
	  <node name="record" pkg="rosbag" type="record" args=" -O $(find xycar_slam)/bags/record.bag /xycar_motor /tf /tf_static /usb_cam/image_raw/compressed /scan /imu" />
	</launch>

	```
	
2. Excute Mapping Luanch File
	```
	$ roslaunch rosbridge_server rosbridge_websocket.launch
	```
	```
	$ ./xycar3dsimulator.x86_64
	```
	```
	$ roslaunch unity_online_mapping.launch
	```

3. Save pbstream
	- Execute Before Exit Mapping Launch File
	```
	$ rosservice call /finish_trajectory 0
	$ rosservice call /write_state “{filename: "File Name", include_unfinished_submaps: “true”}”
	```

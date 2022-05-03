# SLAM Driving

1. Create Mapping Launch File
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
	
2. Excute Mapping Launch File  
	![slam_mapping_simulator](https://user-images.githubusercontent.com/53277342/162653174-353da951-6891-4988-98f3-3c64f1c8cff6.gif)

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
	$ rosservice call /write_state “{filename: "File Name.pbstream", include_unfinished_submaps: “true”}”
	```

4. Get Tracked Pose Bag File  
	![slam_localization](https://user-images.githubusercontent.com/53277342/162653905-153b844e-c2e2-476d-ab3b-0b10c7f213b1.gif)
	
	```
	$ roslaunch rosbridge_server rosbridge_websocket.launch
	```
	
	```
	$ rosbag record -O slam_tracked_pose.bag /tracked_pose
	```

5. Get Way Point in Tracked Pose Bag File  
	![image](https://user-images.githubusercontent.com/53277342/162670973-01372e34-0201-435d-a296-052e100c45cc.png)
	```
	$ roslaunch bag2wp.launch
	```

6. Path Tracking: Stanley Algorithm




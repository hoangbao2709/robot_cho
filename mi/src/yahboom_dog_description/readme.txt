#BUILD
	cd ~/mi
	#colcon build 
	colcon build --packages-select yahboom_dog_description

	#######when open new terminal
	source install/setup.bash

	#xoa build cu
	rm -rf build install log
	
	

#OPEN RVIZ/GAZEBO
	#rviz
		ros2 launch yahboom_dog_description display.launch.py

	#gazebo
		#cach 1:
		ros2 launch yahboom_dog_description gazebo.launch.py
		
		#cach 2:
		#terminal 1:
		gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
		#terminal 2:
		ros2 run gazebo_ros spawn_entity.py -entity yahboom_dog -file /tmp/dog.urdf


#COPY - SHARE TO PHYSICAL MACHINE
	#copy file to vm share: 
	cp name /mnt/hgfs/VMShare/
	#folder:
	cp -r name /mnt/hgfs/VMShare/

	#if can't copy:
	# try this:
	ls /mnt/hgfs/VMShare
	sudo vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other
	#then cp..




#GAZEBO
	#check if node active
	ros2 control list_controllers
	#want to see:
		###legs_position_controller[position_controllers/JointGroupPositionController] active    
		###joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active  
		
		  
#CONTROL IN GAZEBO
	ros2 topic pub ???
	# can find some topic to pub:
	ros2 topic list
	# control pose:
	ros2 topic pub /legs_position_controller/commands   
	std_msgs/msg/Float64MultiArray "{data: [0.2, 0.5, -0, -1.2, 0.5, -0.5.5, 0.3, 0.5, -0.5, -0.3, 0.5, -0.5]}"
	#thu tu trong file controllers.yaml
	
	#xem thong tin trang thai hien tai gazebo
	ros2 service list | grep entity_state
	
	
#URDF
	# 1) Tạo lại URDF từ xacro 
	ros2 run xacro xacro --inorder ~/mi/src/yahboom_dog_description/urdf/yahboom_dog.urdf.xacro -o /tmp/dog.urdf
	# 2) Kiểm tra file không rỗng
	ls -lh /tmp/dog.urdf
	head -n 5 /tmp/dog.urdf
	# 3) Kiểm tra cú pháp URDF
	check_urdf /tmp/dog.urdf
	# create pdf:
	urdf_to_graphviz /tmp/dog.urdf dog_tree




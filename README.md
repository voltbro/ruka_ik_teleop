how to create turtlebot with lewansoul_xarm

1) cd ~/ros_catkin_ws/src/
2) git clone https://github.com/houseofbigseals/lewansoul-xarm
3) cd ..
4) sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=lewansoul_xarm

5) ls /dev/hidraw*
	(
sudo chmod 777 /dev/hidrawX  
where hidrawX is our device
)
6) fix error with urdf_parser_py.urdf somehow
7) sudo cp -r ~/ros_catkin_ws/src/lewansoul-xarm/config /opt/ros/melodic/share/lewansoul_xarm/config/
8) sudo cp -r ~/ros_catkin_ws/src/lewansoul-xarm/urdf /opt/ros/melodic/share/lewansoul_xarm/urdf/
9) rosrun lewansoul_xarm xArm-ros.py
10) there is conflict between current ros on tb and arm node
we have to kill it by systemctl 
sudo systemctl stop turtlebro

11) sudo chmod -R 777 /opt/ros/melodic/share/lewansoul_xarm/config/

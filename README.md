###how to create turtlebot with lewansoul_xarm


``` 
cd ~/ros_catkin_ws/src/
```
``` 
git clone https://github.com/houseofbigseals/lewansoul-xarm
cd ..
```
```
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=lewansoul_xarm
```

```
ls /dev/hidraw*
sudo chmod 777 /dev/hidraw0  
```
where /dev/hidraw0 is our device


```
cd ~/ros_catkin_ws
sudo apt update

sudo apt install python-pip libhidapi-hidraw0 libhidapi-libusb0 python-hid
pip install easyhid

sudo apt-get install python-rosinstall-generator
rosinstall_generator urdfdom_py kdl_parser_py --rosdistro melodic --deps --wet-only --tar > xarm.rosinstall
wstool merge -t src xarm.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=urdfdom_py
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=kdl_parser_py


```
when doing wstool update -t src press 's' everytime, to save data
```
sudo cp -r ~/ros_catkin_ws/src/lewansoul-xarm/config /opt/ros/melodic/share/lewansoul_xarm/config/
```
```
sudo cp -r ~/ros_catkin_ws/src/lewansoul-xarm/urdf /opt/ros/melodic/share/lewansoul_xarm/urdf/
```
```
rosrun lewansoul_xarm xArm-ros.py
```

there is conflict between current ros on tb and arm node
we have to kill it by systemctl 
```
sudo systemctl stop turtlebro
```
```
sudo chmod -R 777 /opt/ros/melodic/share/lewansoul_xarm/config/
```
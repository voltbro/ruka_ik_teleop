## how to use package

#### on turtlebot:
connect xarm, then
```
sudo chmod 777 /dev/hidraw0
rosrun lewansoul_xarm xArm-ros.py
```
you must see smth like this:
```
pi@turtlebro18:~ $ rosrun lewansoul_xarm xArm-ros.py 
HIDDevice:
    /dev/hidraw0 | 483:5750 | MyUSB_HID | LOBOT | 49795F663732
    release_number: 513
    usage_page: 29808
    usage: 452
    interface_number: 0
connected to xArm controller serial number: 49795F663732

```

##### on pc:
connect joystick, then
```
export ROS_MASTER_URI=http://turtlebroXX.local:11311/
roslaunch ruka_ik_teleop joy_joints.launch
```



## how to create turtlebot with lewansoul_xarm


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
sudo chmod -R 777 /opt/ros/melodic/share/lewansoul_xarm/config/
```

```
rosrun lewansoul_xarm xArm-ros.py
```

there is conflict between current ros on tb and arm node
we have to kill it by systemctl 
```
sudo systemctl stop turtlebro
```

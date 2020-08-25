#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from joybro.msg import JoyBro

# before start - we have to connect to master on tbot
# export ROS_MASTER_URI=http://turtlebro18.local:11311/


class JoyHandler(object):
    def __init__(self):
        rospy.init_node('joy_handler', anonymous=True, log_level=rospy.DEBUG)



        rospy.logdebug("joy_handler : init")
        # how sw-triggers must be pushed to choose one of modes
        self.sw_wheels = (0, 1, 1)
        self.sw_joints = (1, 0, 1)
        self.sw_ik_eef = (1, 1, 0)

        # for joint mode
        # to keep states of
        self.current_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.min_joint_states = [-3.14, -1.6, -1.6, -1.6, 0.0]
        self.max_joint_states = [3.14, 1.6, 1.6, 1.6, 3.12]
        self.joints_number = 5
        # for 25.08.20 after calibration in vertical position and closed gripper
        # in # in arm driver measure units
        # -3.14 < arm1 < 3.14
        # -1.6 < arm2 < 1.6
        #  -1.6 < arm3 < 1.6
        # -1.6 < arm4 < 1.6
        # 0 < arm5 < 3.12
        # 0 < gripper < 1.6

        self.current_gripper_states = [0.0]
        self.selected_joint = 0
        self.slider_delta = 10  # in slider measure units
        self.max_gripper_states = [1.6] # in arm driver measure units
        self.min_gripper_states = [0.0]  # in arm driver measure units
        self.current_slider1 = 0
        self.current_slider2 = 0
        self.last_btn3 = 0
        self.last_btn4 = 0
        self.gripper_updated = False
        self.arm_updated = False
        self.gripper_pub = rospy.Publisher("/gripper/move_jp", JointState, queue_size=1)
        self.arm_pub = rospy.Publisher("/arm/move_jp", JointState, queue_size=1)
        # gripper everytime connected to slider1
        # switching between joints is done by buttons btn3 and btn4
        # current arm joint moving by slider2



        self.mode = 'joints'
        # state can be 'joits' - teleoperation for joints
        # 'ik_eef' - teleoperation of eef pose using ik-solver
        # 'wheels' - direct teleoperation of tb motors

        # for wheels mode
        self.wheels_state = 'stop'

        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.25)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.25)
        self.threshold = rospy.get_param('~threshold', 20)

        self.sub = rospy.Subscriber("/joybro", JoyBro, self.callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        rospy.logdebug("joy_handler : end of init")
        rospy.Timer(rospy.Duration(0.05), self.send_data_to_arm)
        self.run()

    def send_data_to_arm(self, event):
        # TODO mb we need to add locks for arrays? check  it
        if self.arm_updated:
            joint_msg = JointState()
            joint_msg.position = self.current_joint_states
            self.arm_pub.publish(joint_msg)
            self.arm_updated = False
            rospy.logdebug("send message {}".format(joint_msg.position))

        if self.gripper_updated:
            gripper_msg = JointState()
            gripper_msg.position = self.current_gripper_states
            self.gripper_pub.publish(gripper_msg)
            self.gripper_updated = False
            rospy.logdebug("send message {}".format(gripper_msg.position))

    def callback(self, data):

        # choose mode
        if (data.sw1, data.sw2, data.sw3) == self.sw_wheels and self.mode != 'wheels':
            self.mode = 'wheels'
            rospy.loginfo("selected mode 'wheels'")
        if (data.sw1, data.sw2, data.sw3) == self.sw_joints and self.mode != 'joints':
            self.mode = 'joints'
            rospy.loginfo("selected mode 'joints'")
        if (data.sw1, data.sw2, data.sw3) == self.sw_ik_eef and self.mode != 'ik_eef':
            self.mode = 'ik_eef'
            rospy.loginfo("selected mode 'ik_eef'")
            pass
        # work in chosen mode

        if self.mode == 'wheels':
            self.wheels_callback(data)
        if self.mode == 'joints':
            self.joints_callback(data)
        if self.mode == 'ik_eef':
            self.ik_eef_callback(data)


    def joints_callback(self, data):

        if data.btn2 == 1 :
            # TODO debounce
            # btn3 moves current joint to +1
            self.selected_joint = 0
            rospy.loginfo("all joints to home position")
            self.current_gripper_states = [0]
            self.current_joint_states = [0, 0, 0, 0, 0]
            self.gripper_updated = True
            self.arm_updated = True

        # check buttons 3 and 4 to choose correct joint
        if data.btn3 != self.last_btn3:
            self.last_btn3 = data.btn3
            if data.btn3 == 1:
                # TODO debounce
                # btn3 moves current joint to +1
                if self.selected_joint == self.joints_number - 1:
                    rospy.loginfo("cannot increase joint, it is already {}".format(
                        self.selected_joint))
                else:
                    rospy.loginfo("joint number {} selected".format(
                        self.selected_joint))
                    self.selected_joint += 1

        if data.btn4 != self.last_btn4:
            self.last_btn4 = data.btn4
            if data.btn4 == 1:
                # TODO debounce
                # btn4 moves current joint to -1
                if self.selected_joint == 0:
                    rospy.loginfo("cannot decrease joint, it is already {}".format(
                        self.selected_joint))
                else:
                    rospy.loginfo("joint number {} selected".format(
                        self.selected_joint))
                    self.selected_joint -= 1




        # check sliders 1 and 2 to move gripper and selected joint
        if abs(data.slider1 - self.current_slider1) > self.slider_delta:

            rospy.logdebug("slider1: {} gripper_states: {}".format(data.slider1, self.current_gripper_states[0]))
            # it means that slider moves really
            self.current_slider1 = data.slider1
            self.current_gripper_states[0] = (float(data.slider1) / 1024.0) * self.max_gripper_states[0]

            rospy.loginfo("gripper moved to {}".format(self.current_gripper_states[0]))
            self.gripper_updated = True



        if abs(data.slider2 - self.current_slider2) > self.slider_delta:
            self.current_slider2 = data.slider2
            rospy.logdebug("slider2: {} joint_states_{}: {}".format(data.slider2, self.selected_joint,
                                                                   self.current_joint_states[self.selected_joint]))

            # it means that slider moves really
            self.current_joint_states[self.selected_joint] = (float(data.slider2) / 1024.0)\
                                                             * self.max_joint_states[self.selected_joint]

            rospy.loginfo("joint number {} moved to {}".format(self.selected_joint,
                                                               self.current_joint_states[self.selected_joint]))


            self.arm_updated = True

        pass

    def ik_eef_callback(self, data):
        pass

    def wheels_callback(self, data):
        linear_vel = 0
        angular_vel = 0

        if self.wheels_state == 'stop' and data.btn3 == 1:
            self.wheels_state = 'move'

        if self.wheels_state == 'move' and data.btn3 == 0:
            twist = Twist()
            self.pub.publish(twist)
            self.wheels_state = 'stop'

        if abs(data.left_y) > self.threshold:
            linear_vel = (data.left_y / 512.0) * self.max_linear_vel

        if abs(data.left_x) > self.threshold:
            angular_vel = -(data.left_x / 512.0) * self.max_angular_vel

        if (self.wheels_state == 'move'):
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            self.pub.publish(twist)

    def run(self):
        rospy.loginfo("joy_handler : go to loop")
        rospy.spin()


if __name__ == '__main__':
    teleop = JoyHandler()


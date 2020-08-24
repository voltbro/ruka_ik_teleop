#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from trac_ik_python.trac_ik import IK


class IKSolver():

    def __init__(self):

        self.ik_solver = IK("world", "link4", timeout=0.1, solve_type="Distance")
        self.start_state = [0.0] * (self.ik_solver.number_of_joints - 1)
        self.bx = self.by = self.bz = 0.001
        self.brx = self.bry = self.brz = 0.1
        self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw = 0, 0, 0, 0, 0, 0, 1
        rospy.init_node("ik_solver")
        self.ik_solution_pub = rospy.Publisher("/arm/move_jp", JointState, queue_size=10)
        self.position_for_controller = JointState()
        self.ROS_main_loop()

        # lb, up = self.ik_solver.get_joint_limits()

    def ik_find_solution(self, x, y, z, qx, qy, qz, qw):

        if self.x != x or self.y != y or self.z != z or self.qx != qx or self.qy != qy or self.qz != qz or self.qw != qw:
            self.sol = self.ik_solver.get_ik(self.curent_js.position, self.x, self.y, self.z,
                                             self.qx, self.qy, self.qz, self.qw,
                                             self.bx, self.by, self.bz,
                                             self.brx, self.bry, self.brz)
            self.x = x
            self.y = y
            self.z = z
            self.qx = qx
            self.qy = qy
            self.qz = qz
            self.qw = qw

            if self.sol != None:
                self.sol = [round(i, 2) for i in self.sol]
                print(self.sol)
                self.position_for_controller.position = self.sol
                self.position_for_controller.position.append(0.0)
                self.ik_solution_pub.publish(self.position_for_controller)
            else:
                print("No solution")

    def ik_callback(self, pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        print(pose)
        self.ik_find_solution(x, y, z, qx, qy, qz, qw)

    def js_cb(self, js):
        self.curent_js.position = js.position[:-1]

    def ROS_main_loop(self):
        self.curent_js = JointState()
        rospy.Subscriber("/l_cont_pose", Pose, self.ik_callback)
        rospy.Subscriber("/arm/measured_js", JointState, self.js_cb)
        rospy.loginfo("Init done. IK plugin is working")
        rospy.spin()


IKSolver()
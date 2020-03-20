#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics


# robot = URDF.from_xml_file("../urdf/test_robot.urdf")
# robot = URDF.from_xml_file("../urdf/ur3.urdf")

class AUBO_robot:
    def __init__(self):
        # rospy.init_node("import_aubo5_from_urdf")
        self.robot = self.init_robot("/home/zy/catkin_ws/src/aubo_robot/aubo_description/urdf/aubo_i5.urdf")
        self.kdl_kin = KDLKinematics(self.robot, "base_link", "wrist3_Link")
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain("base_link", "wrist3_Link")
        # safe angle of UR3

        self.safe_q = [1.3189744444444444, -2.018671111111111, 1.8759755555555557, 2.7850055555555557, 0.17444444444444443, 3.7653833333333337]
        self.q = self.safe_q

    def init_robot(self, filename):
        # print("here")
        robot = URDF.from_xml_file(filename)
        # print("outhere")
        return robot

    def set_q(self, q_list):
        self.q = q_list

    # def get_fk_pose(self):
    #     q = self.q
    #     pose = self.kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
    #     return pose
    #
    # def get_chain(self):
    #     self.tree = kdl_tree_from_urdf_model(self.robot)
    #     self.chain = self.tree.getChain("base_link", "wrist3_Link")

    def get_jacobian(self):
        J = self.kdl_kin.jacobian(self.q)
        return J

    # def get_fk_pose(self):
    def get_ik_pose(self,pose):
        q_ik = self.kdl_kin.inverse(pose)  # inverse kinematics
        # print "q_ik", q_ik
    def test_fk_ik_jacob(self):
        q = self.q
        pose = self.kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        # print pose

        q_ik = self.kdl_kin.inverse(pose, q)  # inverse kinematics
        # print "q_ik", q_ik

        if q_ik is not None:
            pose_sol = self.kdl_kin.forward(q_ik)  # should equal pose
            # print pose_sol

        J = self.kdl_kin.jacobian(q)
        # print 'J:', J
        return J

class aubo_manipulator:
    def __init__(self):
        self.robot = self.init_robot("/home/zy/catkin_ws/src/aubo_robot/aubo_description/urdf/aubo_i5.urdf")
        self.kdl_kin = KDLKinematics(self.robot, "base_link", "wrist3_Link")
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain("base_link", "wrist3_Link")
        self.q = [0,0,0,0,0,0]

    def init_robot(self, filename):
        # print("here")
        robot = URDF.from_xml_file(filename)
        # print("outhere")
        return robot

    def get_jacobian(self,q_list):
        self.q=q_list
        J = self.kdl_kin.jacobian(self.q)
        return J

def main():
    q = [0,0,0,0,0,0]
    aubo5 = AUBO_robot()
    aubo5.set_q(q)
    J=aubo5.get_jacobian()
    J=aubo5.test_fk_ik_jacob()
    aubo5=aubo_manipulator()
    J2=aubo5.get_jacobian(q)
    print(J2-J)
if __name__ == "__main__":
    main()
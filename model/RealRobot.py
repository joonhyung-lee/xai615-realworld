import sys
sys.path.append("..")
import numpy as np 
import math 
from math import pi
import time 
from kinematics.robot import RobotClass
""" FOR MODERN DRIVER """
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
""" FOR ONROBOT RG2 """
from pymodbus.client.sync import ModbusTcpClient

class RealRobot:
    def __init__(self, file_name= "./kinematics/urdf/ur5e/ur5e_onrobot.urdf",
                       base_offset=[0.18, 0, 0.79]):
        self.client      = None
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.arm_pub     = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size = 10)
        self.up_offset   = np.array([0, 0, 0.25])        
        self.file_name   = file_name
        self.base_offset = base_offset
        self.kin_robot   = RobotClass(file_name=self.file_name, 
                                      base_offset=self.base_offset)
        self.client      = None 
        
    def rest_pose(self):
        try:  
            q = [-4.55611547e-01, -1.1069, 2.46218, -1.35530, 1.11438, -1.59319056e-03]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def capture_pose(self):
        try: 
            q = [(-90)/180*math.pi, (-132.46)/180*math.pi, (122.85)/180*math.pi, (99.65)/180*math.pi, (45)/180*math.pi, (-90.02)/180*math.pi]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def capture_pose2(self):
        try: 
            q = [(-180)/180*math.pi, (-132.46)/180*math.pi, (122.85)/180*math.pi, (99.65)/180*math.pi, (45)/180*math.pi, (-90.02)/180*math.pi]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def pnp_prepose(self):
        try: 
            q = [(-77.51)/180*math.pi, (-107.42)/180*math.pi, (116.83)/180*math.pi, (42.62)/180*math.pi, (95.31)/180*math.pi, (6.46)/180*math.pi]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def pnp_grasp1(self):
        try: 
            q = [(-79.97)/180*math.pi, (-27.21)/180*math.pi, (9.76)/180*math.pi, (100.35)/180*math.pi, (87.66)/180*math.pi, (-177.95)/180*math.pi]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def pnp_grasp2(self):
        try: 
            q = [(-79.85)/180*math.pi, (-18.15)/180*math.pi, (9.75)/180*math.pi, (100.33)/180*math.pi, (86.66)/180*math.pi, (-177.96)/180*math.pi]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def start_pose(self):
        try: 
            q = [-3.35916187e-01, -13.90421628e-01,  2.52584599e+00, -1.13542436e+00, 1.23408381e+00, -1.59279665e-03]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def move_trajectory(self, joint_list):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        for i, q in enumerate(joint_list):
            if i==0:
                joint_states = rospy.wait_for_message("joint_states", JointState)
                joints_pos   = joint_states.position
                g.trajectory.points = [
                    JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                    JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
                d=3
            else:
                vel = (q-prev_q) #num_interpol # TODO: CHECK VELOCITY
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=q, velocities=vel,time_from_start=rospy.Duration(d))) 
            prev_q = q
            d+=0.002
        try:
            print("MOVE")
            self.client.send_goal(g)
            self.client.wait_for_result()
        except:
            raise

    def move_slow_trajectory(self, joint_list):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        for i, q in enumerate(joint_list):
            if i==0:
                joint_states = rospy.wait_for_message("joint_states", JointState)
                joints_pos   = joint_states.position
                g.trajectory.points = [
                    JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                    JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
                d=3
            else:
                vel = (q-prev_q) #num_interpol # TODO: CHECK VELOCITY
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=q, velocities=vel,time_from_start=rospy.Duration(d))) 
            prev_q = q
            d+=0.007
        try:
            print("MOVE")
            self.client.send_goal(g)
            self.client.wait_for_result()
        except:
            raise

    def main(self, joint_list=None): 
        try: 
            # rospy.init_node("test_move", anonymous=True, disable_signals=True)
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            parameters = rospy.get_param(None)
            index = str(parameters).find('prefix')
            if (index > 0):
                prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
                for i, name in enumerate(self.JOINT_NAMES):
                    self.JOINT_NAMES[i] = prefix + name
            self.move_trajectory(joint_list=joint_list)

            # order = input("Continue? y/n: ")
            # if order=="y": 
            #     self.move_trajectory(joint_list=joint_list)
            # else: 
            #     print("Stop")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise


    def main_slow(self, joint_list=None): 
        try: 
            # rospy.init_node("test_move", anonymous=True, disable_signals=True)
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            parameters = rospy.get_param(None)
            index = str(parameters).find('prefix')
            if (index > 0):
                prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
                for i, name in enumerate(self.JOINT_NAMES):
                    self.JOINT_NAMES[i] = prefix + name
            self.move_slow_trajectory(joint_list=joint_list)

            # order = input("Continue? y/n: ")
            # # if order=="y": 
            #     self.move_slow_trajectory(joint_list=joint_list)
            # else: 
            #     print("Stop")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def move_capture_pose(self): 
        try: 
            # rospy.init_node("test_move", anonymous=True, disable_signals=True)
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            parameters = rospy.get_param(None)
            index = str(parameters).find('prefix')
            if (index > 0):
                prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
                for i, name in enumerate(self.JOINT_NAMES):
                    self.JOINT_NAMES[i] = prefix + name
            self.capture_pose()

            # order = input("Continue? y/n: ")
            # if order=="y": 
            #     print("Press Y")
            #     self.capture_pose()
            # else: 
            #     print("Stop")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def move_capture_pose2(self): 
        try: 
            # rospy.init_node("test_move", anonymous=True, disable_signals=True)
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            parameters = rospy.get_param(None)
            index = str(parameters).find('prefix')
            if (index > 0):
                prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
                for i, name in enumerate(self.JOINT_NAMES):
                    self.JOINT_NAMES[i] = prefix + name
            self.capture_pose2()

            # order = input("Continue? y/n: ")
            # if order=="y": 
            #     print("Press Y")
            #     self.capture_pose2()
            # else: 
            #     print("Stop")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def move_pnp_prepose(self): 
        try: 
            # rospy.init_node("test_move", anonymous=True, disable_signals=True)
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            parameters = rospy.get_param(None)
            index = str(parameters).find('prefix')
            if (index > 0):
                prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
                for i, name in enumerate(self.JOINT_NAMES):
                    self.JOINT_NAMES[i] = prefix + name
            order = input("Continue? y/n: ")
            if order=="y": 
                print("Press Y")
                self.pnp_prepose()
            else: 
                print("Stop")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def move_pnp_grasp1(self): 
        try: 
            # rospy.init_node("test_move", anonymous=True, disable_signals=True)
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            parameters = rospy.get_param(None)
            index = str(parameters).find('prefix')
            if (index > 0):
                prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
                for i, name in enumerate(self.JOINT_NAMES):
                    self.JOINT_NAMES[i] = prefix + name
            order = input("Continue? y/n: ")
            if order=="y": 
                print("Press Y")
                self.pnp_grasp1()
            else: 
                print("Stop")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def move_pnp_grasp2(self): 
        try: 
            # rospy.init_node("test_move", anonymous=True, disable_signals=True)
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            parameters = rospy.get_param(None)
            index = str(parameters).find('prefix')
            if (index > 0):
                prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
                for i, name in enumerate(self.JOINT_NAMES):
                    self.JOINT_NAMES[i] = prefix + name
            order = input("Continue? y/n: ")
            if order=="y": 
                print("Press Y")
                self.pnp_grasp2()
            else: 
                print("Stop")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def move_rest_pose(self): 
        try: 
            # rospy.init_node("test_move", anonymous=True, disable_signals=True)
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            parameters = rospy.get_param(None)
            index = str(parameters).find('prefix')
            if (index > 0):
                prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
                for i, name in enumerate(self.JOINT_NAMES):
                    self.JOINT_NAMES[i] = prefix + name
            self.rest_pose()

            # order = input("Continue? y/n: ")
            # if order=="y": 
            #     print("Press Y")
            #     self.rest_pose()
            # else: 
            #     print("Stop")
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def gripper_open(self, force,width,onrobot_ip='192.168.1.1'):
        graspclient = ModbusTcpClient(onrobot_ip)
        slave = 65
        graspclient.write_registers(0,[force,width,1],unit=slave)
        time.sleep(1)
        graspclient.close()
    
    def gripper_close(self, force,width,onrobot_ip='192.168.1.1'):
        graspclient = ModbusTcpClient(onrobot_ip)
        slave = 65
        graspclient.write_registers(0,[force,width,1],unit=slave)
        time.sleep(1)
        graspclient.close()


import sys
sys.path.append("..")
import numpy as np 
import math 
from math import pi
import time 
import copy
""" FOR MODERN DRIVER """
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
""" FOR ONROBOT RG2 """
from pymodbus.client.sync import ModbusTcpClient

# TODO: Implement about RealRobot Class.
class RealRobot:
    def __init__(self):
        self.client = None
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.arm_pub     = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size = 10)

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

        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
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

        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
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

    def execute_arm_speed(self, traj, speed_limit):
        try:
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            """ Initialize """
            self.move_arm_speed(traj, speed_limit)
            print("Finish plan")

        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise      

    def move_arm_speed(self, traj:JointTrajectory, speed_limit):
        joint_pos1 = rospy.wait_for_message("joint_states", JointState).position
        rospy.sleep(0.1)
        joint_pos2 = rospy.wait_for_message("joint_states", JointState).position

        joint_pos1, joint_pos2 = np.array(joint_pos1), np.array(joint_pos2)

        diff = np.linalg.norm(joint_pos2 - joint_pos1)

        
        if diff > 5e-3:            
            raise Exception(f"Loose Connection, diff:{diff}")
        assert np.linalg.norm(joint_pos2) < 20

        try: 
            g = FollowJointTrajectoryGoal()
            g.trajectory = copy.deepcopy(traj)
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position

            if np.linalg.norm(joints_pos) > 10:
                print("Loose Connection")
                return None

            init_point = JointTrajectoryPoint()
            init_point.positions = np.array(joints_pos)
            init_point.time_from_start = rospy.Duration.from_sec(0.0)
            init_point.velocities = [0 for _ in range(6)]
            g.trajectory.points.insert(0, copy.deepcopy(init_point))

            q_list = []
            time_list = []
            for point in g.trajectory.points:
                q_list.append(point.positions)
                time_list.append(point.time_from_start.to_sec())

            print(q_list)

            for i in range(len(q_list)-1):
                q_before = np.array(q_list[i])
                q_after  = np.array(q_list[i+1])
                print(q_after, q_before)
                time_before = time_list[i]
                time_after = time_list[i+1]

                diff_q = q_after - q_before
                diff_time = time_after - time_before
                print(f"diff_q:{diff_q}")
                print(f"diff_time:{diff_time}")
                speed = np.linalg.norm(diff_q)/diff_time
                print(speed)
                if speed >= speed_limit:
                    raise Exception(f"Speed is too fast: {speed} < {speed_limit}")
                    

            self.client.send_goal(g)
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise          


def euclidean_dist(point1, point2):
    return math.sqrt(sum([math.pow(point1[i] - point2[i], 2) for i in range(len(point1))]))

def get_desired_time(start_pos, target_pos, desired_vel): 
    length = euclidean_dist(start_pos, target_pos)    
    desired_time = length/desired_vel
    return desired_time

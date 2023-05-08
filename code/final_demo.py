# %%
import sys 
sys.path.append("..")
import cv2 
import numpy as np 
import time
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt 
print("Done.")

# %%
# class UR_Test():
#     def __init__(self):
#         self.tick = 0
#         self.joint_list = None 
#         self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)

#         tic_temp=0 

#         while self.tick<2: 
#             time.sleep(1e-3)
#             tic_temp=tic_temp+1

#             if tic_temp>5000: 
#                 print ("[ERROR] GET JOINTS")
#                 break 
            
#     def joint_callback(self, joint_msg):
#         """
#             Get joint values about each joint.
#         """
#         self.tick+=1 
#         self.joint_list = joint_msg 

# %%
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

import copy
import numpy as np

from numpy import pi as PI
DEG90 = PI/2

class UR(object):
    def __init__(self,):
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.client      = None
        
   
    def move_arm_speed(self, traj:JointTrajectory, speed_limit):
        try: 
            g = FollowJointTrajectoryGoal()
            g.trajectory = copy.deepcopy(traj)
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position

            init_point = JointTrajectoryPoint()
            init_point.positions = joints_pos
            init_point.time_from_start = rospy.Duration.from_sec(0.1)
            init_point.velocities = [0 for _ in range(6)]
            g.trajectory.points.insert(0, init_point)

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

                speed = np.linalg.norm(diff_q)/diff_time
                print(speed)
                assert speed <= speed_limit
            
            

            self.client.send_goal(g)
            # self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise  

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

    def execute_arm_time(self, joints, movetime):
        try:
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            """ Initialize """
            self.move_arm_time(joints, movetime)
            print("Finish plan")

        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise      

# %%
rospy.init_node('final_demo')
# ur_test = UR_Test()

# %%
import mujoco
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("..")
from model.mujoco_parser import MuJoCoParserClass
from model.util import sample_xyzs,rpy2r,r2quat
np.set_printoptions(precision=2,suppress=True,linewidth=100)
plt.rc('xtick',labelsize=6); plt.rc('ytick',labelsize=6)
print ("MuJoCo version:[%s]"%(mujoco.__version__))

# %%
xml_path = '../asset/ur5e/scene_ur5e_rg2_obj.xml'
env = MuJoCoParserClass(name='UR5e with RG2 gripper',rel_xml_path=xml_path,VERBOSE=True)
obj_names = [body_name for body_name in env.body_names
             if body_name is not None and (body_name.startswith("obj_"))]
n_obj = len(obj_names)
# Place objects in a row
xyzs = sample_xyzs(n_sample=n_obj,
                   x_range=[0.45,1.65],y_range=[-0.38,0.38],z_range=[0.81,0.81],min_dist=0.2)
colors = np.array([plt.cm.gist_rainbow(x) for x in np.linspace(0,1,n_obj)])
for obj_idx,obj_name in enumerate(obj_names):
    jntadr = env.model.body(obj_name).jntadr[0]
    env.model.joint(jntadr).qpos0[:3] = xyzs[obj_idx,:]
    geomadr = env.model.body(obj_name).geomadr[0]
    env.model.geom(geomadr).rgba = colors[obj_idx] # color

# Move tables and robot base
env.model.body('base_table').pos = np.array([0,0,0])
env.model.body('front_object_table').pos = np.array([1.05,0,0])
env.model.body('side_object_table').pos = np.array([0,-0.85,0])
env.model.body('base').pos = np.array([0,0,0.8])
print ("Ready.")

# %%
joint_names = env.rev_joint_names[:6]
idxs_forward = [env.model.joint(joint_name).qposadr[0] for joint_name in env.joint_names[:6]]
idxs_jacobian = [env.model.joint(joint_name).dofadr[0] for joint_name in env.joint_names[:6]]
list1, list2 = env.ctrl_joint_idxs, idxs_forward
idxs_step = []
for i in range(len(list2)):
    if list2[i] not in list1:
        idxs_step.append(list1.index(list2[i]))
        

# %%
from numpy import pi as PI

cature_q = np.array([-90, -90, 90, 90, 55, -90], dtype=np.float32) / 180 * PI
env.forward(q=cature_q,joint_idxs=idxs_forward)
print(env.get_q(joint_idxs=idxs_jacobian))

joint_names = env.rev_joint_names[:6]

p_cam = env.get_p_body("camera_center") - env.get_p_body("base")

R_world = env.get_R_body('camera_center')
R_world


# %%
from model.util import rpy2r, r2rpy, pr2t
from demo_realworld_perception import get_center_position, Translation, Rotation_X, Rotation_Y, HT_matrix
import numpy as np
import math

# rpy = r2rpy(R_world)
# R_cam = rpy2r([-rpy[0], PI/2 - rpy[1], PI-rpy[2]])

rotation_x   = Rotation_X(-math.pi)
rotation_y   = Rotation_Y(-math.pi/180*55)
rotation_mat = np.dot(rotation_x, rotation_y)

robot = UR()

q = cature_q
q_traj = JointTrajectory()

point = JointTrajectoryPoint()
point.positions = q
point.velocities = [0 for _ in range(6)]
point.time_from_start = rospy.Duration.from_sec(20)

q_traj.points.append(point)
robot.execute_arm_speed(q_traj, speed_limit=0.1)

perception_path =  "/home/terry/Rilab/sOftrobot/UnseenObjectClustering"
center_position_array, radius = get_center_position(perception_path, p_cam, rotation_mat, clean_scale = 3, VIZ=True)
tcp_target = np.array([center_position_array[0], center_position_array[1], 0.04]) + env.get_p_body("base")

# %%
import math
PI = math.pi

err_th=1e-3


joint_idx = np.asarray([ i for i in range(16,22)], 'int32')

env.init_viewer(viewer_title='IK using UR',viewer_width=1200,viewer_height=800,viewer_hide_menus=True)
env.update_viewer(azimuth=80,distance=2.5,elevation=-30,lookat=[0,0,1.5])
env.update_viewer(VIS_TRANSPARENT=True) # transparent
env.reset() # reset

env.forward(q=cature_q,joint_idxs=idxs_forward)


delta_p =  (env.get_p_body("tcp_link")-env.get_p_body('wrist_3_link'))
R_ = env.get_R_body("wrist_3_link")
p_offset = R_.T @ delta_p
wrist3_target = tcp_target - p_offset[[1,0,2]]


# Set (multiple) IK targets
ik_body_names = ['tcp_link','wrist_3_link']
ik_p_trgts = [tcp_target,
              wrist3_target]
ik_R_trgts = [rpy2r(np.array([0, 1, -0.5])*PI ),
              rpy2r([0,0,0])]
IK_Ps = [True,True]
IK_Rs = [True,False]

# Loop
q = env.get_q(joint_idxs=idxs_jacobian)

# q = env.data.qpos[16:22]
imgs,img_ticks,max_tick = [],[],1000
qs = []
while (env.tick < max_tick) and env.is_viewer_alive():
    # Numerical IK
    J_aug,err_aug = [],[]
    for ik_idx,ik_body_name in enumerate(ik_body_names):
        p_trgt,R_trgt = ik_p_trgts[ik_idx],ik_R_trgts[ik_idx]
        IK_P,IK_R = IK_Ps[ik_idx],IK_Rs[ik_idx]
        J,err = env.get_ik_ingredients(
            body_name=ik_body_name,p_trgt=p_trgt,R_trgt=R_trgt,IK_P=IK_P,IK_R=IK_R)
        if (J is None) and (err is None): continue
        if len(J_aug) == 0:
            J_aug,err_aug = J,err
        else:
            J_aug   = np.concatenate((J_aug,J),axis=0)
            err_aug = np.concatenate((err_aug,err),axis=0)
    dq = env.damped_ls(J_aug,err_aug,stepsize=1,eps=1e-1,th=5*np.pi/180.0)

    qs.append(q)
    err_norm = np.linalg.norm(err_aug)
    if err_norm < err_th:
        break

    # Update q and FK
    q = q + dq[idxs_jacobian]
    env.forward(q=q,joint_idxs=idxs_forward)

    p_contacts,f_contacts,geom1s,geom2s = env.get_contact_info(must_exclude_prefix="obj_")

    geom1s_ = [obj_ for obj_ in geom1s if obj_ not in ["rg2_gripper_finger1_finger_tip_link","rg2_gripper_finger2_finger_tip_link"]]
    geom2s_ = [obj_ for obj_ in geom2s if obj_ not in ["rg2_gripper_finger1_finger_tip_link","rg2_gripper_finger2_finger_tip_link"]]

    if len(geom1s_) > 0:
        print(f"Collision with {geom1s_[0]} and {geom2s_[0]}")
        break

    # Render
    for ik_idx,ik_body_name in enumerate(ik_body_names):
        p_trgt,R_trgt = ik_p_trgts[ik_idx],ik_R_trgts[ik_idx]
        IK_P,IK_R = IK_Ps[ik_idx],IK_Rs[ik_idx]
        if (IK_P is None) and (IK_R is None): continue
        env.plot_T(p=env.get_p_body(body_name=ik_body_name),R=env.get_R_body(body_name=ik_body_name),
                   PLOT_AXIS=IK_R,axis_len=0.2,axis_width=0.01,
                   PLOT_SPHERE=IK_P,sphere_r=0.05,sphere_rgba=[1,0,0,0.9])
        env.plot_T(p=p_trgt,R=R_trgt,
                   PLOT_AXIS=IK_R,axis_len=0.2,axis_width=0.01,
                   PLOT_SPHERE=IK_P,sphere_r=0.05,sphere_rgba=[0,0,1,0.9])
    env.plot_T(p=[0,0,0],R=np.eye(3,3),PLOT_AXIS=True,axis_len=1.0)
    env.render()
    # Print and save image 
    if (env.tick)%(max_tick//10)==0 or (env.tick==1):
        print ("[%d/%d] IK error:[%.4f]"%(env.tick,max_tick,np.linalg.norm(err_aug)))
        img = env.grab_image()
        imgs.append(img)
        img_ticks.append(env.tick)

qs_array = np.array(qs)

# Close viewers
env.close_viewer()
print ("Done.")



# %%


unit_time = 1
track_time = 0

q_before = cature_q
speed_limit = 0.1 # speed limit of joint velocity (Must be fixed to consider EE velocity instead)

for i, qs in enumerate(qs_array):    
    delta_q = np.linalg.norm(qs - q_before)

    unit_time = max(delta_q/0.2, unit_time)
    
    track_time = track_time + unit_time
    point = JointTrajectoryPoint()
    point.positions = qs
    point.velocities = [0 for _ in range(6)]
    point.time_from_start = rospy.Duration.from_sec(track_time)
    q_traj.points.append(point)
    
    q_before = qs

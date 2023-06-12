# %%
import sys 
sys.path.append("../../")
import cv2 
import numpy as np 
import time
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt 
print("Done.")

# %%
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

import copy
import numpy as np

PI = 3.14
DEG90 = PI/2

class UR(object):
    def __init__(self,):
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.client      = None
        
   
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

# %%
import mujoco
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("../../")
from model.mujoco_parser import MuJoCoParserClass
from model.util import sample_xyzs,rpy2r,r2quat
np.set_printoptions(precision=2,suppress=True,linewidth=100)
plt.rc('xtick',labelsize=6); plt.rc('ytick',labelsize=6)
print ("MuJoCo version:[%s]"%(mujoco.__version__))

# %%
xml_path = '../../asset/ur5e/scene_ur5e_rg2_obj.xml'
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
    if list2[i] in list1:
        idxs_step.append(list1.index(list2[i]))
        

# %%
""" FOR ONROBOT RG2 """
from pymodbus.client.sync import ModbusTcpClient
""" FOR MODERN DRIVER """
import roslib; roslib.load_manifest('ur_driver')
import rospy
import sys
from model.gripper import openGrasp, closeGrasp, resetTool

graspclient = ModbusTcpClient('192.168.0.22') 
# resetTool(graspclient)
# openGrasp(force=200, width=1000, graspclient=graspclient)
# time.sleep(1)

from model.util import rpy2r, r2rpy, pr2t
from demo_realworld_perception import get_center_position, Translation, Rotation_X, Rotation_Y, HT_matrix
import numpy as np
import math

capture_q = np.array([-90, -90, 90, 90, 70, -90], dtype=np.float32) / 180 * PI
env.forward(q=capture_q,joint_idxs=idxs_forward)
print(env.get_q(joint_idxs=idxs_jacobian))

joint_names = env.rev_joint_names[:6]

p_cam = env.get_p_body("camera_center") - env.get_p_body("base") + np.array([-0.03, 0.03,0])
R_world = env.get_R_body('camera_center')

rotation_mat = np.eye(4)
Transform_rel = rpy2r(np.array([0, -0.5, -0.5])*PI)
rotation_mat[:3,:3] =  R_world @ Transform_rel

robot = UR()

q = copy.deepcopy(capture_q)
q_traj = JointTrajectory()

point = JointTrajectoryPoint()
point.positions = q
point.velocities = [0 for _ in range(6)]
point.time_from_start = rospy.Duration.from_sec(10)

q_traj.points.append(point)
robot.execute_arm_speed(q_traj, speed_limit=1.0)
robot.client.wait_for_result()

#%%
from demo_realworld_perception import run_camera, project_XY, project_YZ
# perception_path =  "/home/terry/Rilab/sOftrobot/UnseenObjectClustering"
perception_path = "/home/rilab-ur/UnseenObjectClustering"

VIZ = True
cleaned_point_cloud_list = run_camera(perception_path, camera_p = p_cam, rotation_mat=rotation_mat, clean_scale=3)

# %%
edge = 0.01

def detect_hough(image_path):
    import cv2
    import numpy as np

    # Load image
    partial_image = cv2.imread(image_path, 0)

    # Apply Gaussian blur to reduce noise
    blurred_image = cv2.GaussianBlur(partial_image, (33,33), 250)

    dp = 10.0  # Inverse ratio of the accumulator resolution to the image resolution (1 means same resolution)
    min_dist = 500  # Minimum distance between the centers of the detected circles
    param1 = 10  # Upper threshold for the internal Canny edge detector
    param2 = 5  # Threshold for center detection (lower value for more circles)
    min_radius = 70  # Minimum circle radius
    max_radius = 140  # Maximum circle radius (0 for no limit)

    circles = cv2.HoughCircles(blurred_image, cv2.HOUGH_GRADIENT, dp, min_dist,
                               param1=param1, param2=param2,
                               minRadius=min_radius, maxRadius=max_radius)


    # Draw detected circles on the original image
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]
            cv2.circle(blurred_image, center, radius, (0, 255, 0), 2)
    else: print("No Circles!")

    # Display the result
    plt.imshow(blurred_image, cmap='gray')
    return partial_image, center, radius

z_hard = 0.07
edge = 0.02

center_position_list = []
for cleaned_point_cloud in cleaned_point_cloud_list:
    image_path = "XY.png"
    ax, fig, XY_info = project_XY(cleaned_point_cloud, edge, SAVE=image_path)
    plt.show()

    partial_image, center_pixel, radius_pixel = detect_hough("XY.png")
    scale_rate = 2*(XY_info['radius']+XY_info['edge'])/partial_image.shape[0]

    center_x = center_pixel[0] * scale_rate + XY_info['center_x'] - (XY_info['radius']+edge)
    center_y = -1 * center_pixel[1] * scale_rate + XY_info['center_y'] + (XY_info['radius']+edge)
    radius = radius_pixel * scale_rate

    ax, fig, XY_info = project_XY(cleaned_point_cloud, edge=edge, SAVE=None)
    circle = plt.Circle((center_x, center_y), radius, fill=False, color='r', lw=5)
    ax.add_artist(circle)
    plt.show()

    center_position_list.append(np.array([center_y, center_x, z_hard]))


# %%
# number of objects
assert len(center_position_list) == 4

target_idx = np.argmin(np.stack(center_position_list)[:,1])
pick_position_array = center_position_list[target_idx]
place_position_array = center_position_list[1-target_idx]

pick_target = pick_position_array + env.get_p_body("base")
place_target = place_position_array + env.get_p_body("base")

pick_target, place_target


# %%
# Solve IK

err_th=5e-3


joint_idx = np.asarray([ i for i in range(16,22)], 'int32')

env.init_viewer(viewer_title='IK using UR',viewer_width=1200,viewer_height=800,viewer_hide_menus=True)
env.update_viewer(azimuth=80,distance=2.5,elevation=-30,lookat=[0,0,1.5])
env.update_viewer(VIS_TRANSPARENT=True) # transparent
env.reset() # reset

env.forward(q=capture_q,joint_idxs=idxs_forward)

delta_p =  (env.get_p_body("tcp_link")-env.get_p_body('wrist_3_link'))
R_ = env.get_R_body("wrist_3_link")
p_offset = R_.T @ delta_p
wrist3_target = pick_target - p_offset[[1,0,2]]


# Set (multiple) IK targets
ik_body_names = ['tcp_link','wrist_3_link']
ik_p_trgts_0 = [pick_target+np.array([-0.1,0,0.03]),
              wrist3_target+np.array([-0.1,0,0.03])]
ik_p_trgts_1 = [pick_target+np.array([-0.1,0,0]),
              wrist3_target+np.array([-0.1,0,0])]
ik_p_trgts_2 = [pick_target,
              wrist3_target]
ik_p_trgts_3 = [pick_target+np.array([0,0,0.25]),
              wrist3_target+np.array([0,0,0.25])]

wrist3_target = place_target - p_offset[[1,0,2]]
ik_p_trgts_4 = [place_target+np.array([0,0,0.25]),
              wrist3_target+np.array([0,0,0.25])]

ik_p_trgts_5 = [place_target+np.array([0,0,0.16]),
              wrist3_target+np.array([0,0,0.16])]

ik_p_trgts_6 = [place_target+np.array([0,0,0.14]),
              wrist3_target+np.array([0,0,0.14])]



ik_R_trgts = [rpy2r(np.array([0, 1, -0.5])*PI ),
              rpy2r([0,0,0])]
IK_Ps = [True,True]
IK_Rs = [True,False]

ik_p_trgts_list = [ik_p_trgts_0, ik_p_trgts_1, ik_p_trgts_2, ik_p_trgts_3, ik_p_trgts_4, ik_p_trgts_5, ik_p_trgts_6]
grasp_list = [None, None, None, 'close', None, None, None, 'open']

# Loop
q = env.get_q(joint_idxs=idxs_jacobian)

# q = env.data.qpos[16:22]
imgs,img_ticks,max_tick = [],[],500
qs = [capture_q]

for ik_p_trgts in ik_p_trgts_list:
    while (env.tick < max_tick) and env.is_viewer_alive():
        # Numerical IK
        J_aug,err_aug = [],[]
        for ik_idx,ik_body_name in enumerate(ik_body_names):
            p_trgt,R_trgt = ik_p_trgts[ik_idx],ik_R_trgts[ik_idx]
            IK_P,IK_R = IK_Ps[ik_idx],IK_Rs[ik_idx]
            J,err = env.get_ik_ingredients(
                body_name=ik_body_name,p_trgt=p_trgt,R_trgt=R_trgt,IK_P=IK_P,IK_R=IK_R, w_weight=1)
            if (J is None) and (err is None): continue
            if len(J_aug) == 0:
                J_aug,err_aug = J,err
            else:
                J_aug   = np.concatenate((J_aug,J),axis=0)
                err_aug = np.concatenate((err_aug,err),axis=0)
        dq = env.damped_ls(J_aug,err_aug,stepsize=1,eps=1e-2,th=5*np.pi/180.0)
        # print(dq)
        err_norm = np.linalg.norm(err_aug)
        if err_norm < err_th:
            break

        # Update q and FK
        q = q + dq[idxs_jacobian]
        env.forward(q=q,joint_idxs=idxs_forward)

        p_contacts,f_contacts,geom1s,geom2s = env.get_contact_info(must_exclude_prefix="obj_")

        geom1s_ = [obj_ for obj_ in geom1s if obj_ not in ["ur_rg2_gripper_finger1_finger_tip_link_collision","ur_rg2_gripper_finger2_finger_tip_link_collision"]]
        geom2s_ = [obj_ for obj_ in geom2s if obj_ not in ["ur_rg2_gripper_finger1_finger_tip_link_collision","ur_rg2_gripper_finger2_finger_tip_link_collision"]]

        if len(geom1s_) > 0:
            print(f"Collision with {geom1s_[0]} and {geom2s_[0]}")
            q = q - dq[idxs_jacobian] * 30
            # break

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

    qs.append(q)
qs_array = np.array(qs)

# Close viewers
env.close_viewer()
print ("Done.")


# %%
unit_time_base = 1.0
track_time = 0

q_curr = rospy.wait_for_message("joint_states", JointState).position
env.forward(q=q_curr,joint_idxs=idxs_forward)
x_before = env.get_p_body("tcp_link")

speed_limit = 0.15 # speed limit of joint velocity (Must be fixed to consider EE velocity instead)

resetTool(graspclient)
openGrasp(force=200, width=1000, graspclient=graspclient)
time.sleep(1)

for i, qs in enumerate(qs_array):    
    q_traj = JointTrajectory()

    env.forward(q=qs,joint_idxs=idxs_forward)
    x_curr = env.get_p_body("tcp_link")
    delta_x = np.linalg.norm(x_curr - x_before)

    unit_time = max(delta_x/(speed_limit), unit_time_base)
    print(unit_time)
    track_time = track_time + unit_time
    point = JointTrajectoryPoint()
    point.positions = qs
    point.velocities = [0 for _ in range(6)]
    point.time_from_start = rospy.Duration.from_sec(track_time)
    q_traj.points.append(point)
    
    x_before = x_curr

    robot.execute_arm_speed(q_traj, speed_limit=1.6)
    robot.client.wait_for_result()
    time.sleep(0.5)

    if grasp_list[i] is None:
        continue
    elif grasp_list[i].lower() == 'open':
        openGrasp(force=200, width=1000, graspclient=graspclient)
    elif grasp_list[i] == 'close':
        closeGrasp(force=200, width=100, graspclient=graspclient)
    else:
        raise ValueError('grasp_list must be None, open, or close')



# %%

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
class UR_Test():
    def __init__(self):
        self.tick = 0
        self.joint_list = None 
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        tic_temp=0 

        while self.tick<2: 
            time.sleep(1e-3)
            tic_temp=tic_temp+1

            if tic_temp>5000: 
                print ("[ERROR] GET JOINTS")
                break 
            
    def joint_callback(self, joint_msg):
        """
            Get joint values about each joint.
        """
        self.tick+=1 
        self.joint_list = joint_msg 

# %%
rospy.init_node('final_demo')
ur_test = UR_Test()

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
env.forward()
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

cature_q = np.array([-90, -90, 90, 90, 55, 0], dtype=np.float32) / 180 * PI
env.forward(q=cature_q, joint_idxs = [0,1,2,3,4,5])

joint_names = env.rev_joint_names[:6]

p_cam = env.get_p_body("camera_center") - env.get_p_body("base")

R_world = env.get_R_body('camera_center')
R_world


# %%
from model.util import rpy2r, r2rpy, pr2t
rpy = r2rpy(R_world)
R_cam = rpy2r([-rpy[0], PI/2 - rpy[1], PI-rpy[2]])

# %%
from demo_realworld_perception import get_center_position, Translation, Rotation_X, Rotation_Y, HT_matrix
import numpy as np
import math

perception_path =  "/home/terry/Rilab/sOftrobot/UnseenObjectClustering"
center_position_array, radius = get_center_position(perception_path, p_cam, pr2t(np.zeros(3), R_cam), clean_scale = 3, VIZ=True)

# %%

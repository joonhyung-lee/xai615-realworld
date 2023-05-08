# %%


# %%
from demo_realworld_perception import get_center_position, Translation, Rotation_X, Rotation_Y, HT_matrix
import numpy as np
import math

perception_path =  "/home/terry/Rilab/sOftrobot/UnseenObjectClustering"

# Relative position to Base
camera_p = np.array([ 0.33107886, -0.25199999,  1.08883276])

# Calibration
rotation_x   = Rotation_X(-math.pi)
rotation_y   = Rotation_Y(-math.pi/180*55)
rotation_mat = np.dot(rotation_x, rotation_y)

position_mat = Translation(*(camera_p.tolist()))
transform_mat= HT_matrix(rotation_mat, position_mat)

center_position_array, radius = get_center_position(perception_path, camera_p, rotation_mat, clean_scale = 3, VIZ=True)

# %%

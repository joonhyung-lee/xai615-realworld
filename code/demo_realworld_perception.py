# %% [markdown]
# ### Test `Unseen Object Clustering (UCN)` Network

# %% [markdown]
# ### Run the shell script of `UCN Network`

# %%
perception = "/home/terry/Rilab/sOftrobot/UnseenObjectClustering"

# %%
CAPTURE = True

import subprocess
import os
import math
import matplotlib.pyplot as plt
import numpy as np


def capture_camera():
    if CAPTURE:
        if os.path.exists(f"{perception}/label_data.npy"):
            os.remove(f"{perception}/label_data.npy")

        if os.path.exists(f"{perception}/depth_data.npy"):
            os.remove(f"{perception}/depth_data.npy")

        subprocess.run([f"cd {perception}; ./experiments/scripts/ros_seg_rgbd_add_test_segmentation_realsense.sh $GPU_ID 0"], shell=True)

# %%

# %%
def Rotation_X(rad):
    roll = np.array([[1, 	       0, 	      0,    0],
             		 [0, np.cos(rad), -np.sin(rad), 0],
             		 [0, np.sin(rad),  np.cos(rad), 0],
             		 [0,		   0,	      0,    0]])
    return roll 


def Rotation_Y(rad):
    pitch = np.array([[np.cos(rad), 0, np.sin(rad), 0],
              		  [0,		    1, 	         0, 0],
              		  [-np.sin(rad),0, np.cos(rad), 0],
              		  [0, 		    0, 	         0, 0]])
    return pitch


def Rotation_Z(rad):
    yaw = np.array([[np.cos(rad), -np.sin(rad),  0, 0],
         	        [np.sin(rad),  np.cos(rad),  0, 0],
              		[0, 			         0,  1, 0],
             		[0, 			         0,  0, 0]])
    return yaw 

def Translation(x , y, z):
    Position = np.array([[0, 0, 0, x],
                         [0, 0, 0, y],
                         [0, 0, 0, z],
                         [0, 0, 0, 1]])
    return Position


def HT_matrix(Rotation, Position):
    Homogeneous_Transform = Rotation + Position
    return Homogeneous_Transform


# %%




# %%
def camera_to_base(transform_mat, points):
    ones = np.ones((len(points),1))
    points = np.concatenate((points,ones),axis=1)
    t_points = points.T
    t_transformed_ponints = np.dot(transform_mat,t_points)
    transformed_ponints = t_transformed_ponints.T
    xyz = transformed_ponints[:,0:3]
    return xyz

# %%
def clean_point_cloud(single_cluster,clean_scale):
    # assume point_cloud is the point cloud as a numpy array with shape (n_points, 3)

    single_cluster = single_cluster[:,:3]
    # calculate the median of each dimension
    medians = np.median(single_cluster, axis=0)

    # calculate the median absolute deviation (MAD) of each dimension
    mad = np.median(np.abs(single_cluster - medians), axis=0)

    # set a threshold for outliers based on the MAD
    threshold = clean_scale * mad

    # calculate the distance of each point from the medians
    distances = np.abs(single_cluster - medians)

    # identify the outliers by comparing the distances to the threshold
    outliers = np.any(distances > threshold, axis=1)

    # remove the outliers from the point cloud
    cleaned_point_cloud = single_cluster[~outliers]
    return cleaned_point_cloud




# %%
def cluster_info_2_cleaned_point_cloud(label_pixel, depth_pixel, transform_mat, clean_scale=3):
    # clustering_num=len(np.unique(label_pixel))

    cluster_list = []
    # Total cluster loop
    for cluster_idx in np.unique(label_pixel):
        if cluster_idx ==0: continue

        # Random color 
        # color = random_color_gen()
        cluster_xlst, cluster_ylst = np.where(label_pixel==cluster_idx)

        clusters = []
        for idx, (x,y,z) in enumerate(zip(depth_pixel[2,cluster_xlst, cluster_ylst], \
                                            depth_pixel[0,cluster_xlst, cluster_ylst], \
                                            depth_pixel[1,cluster_xlst, cluster_ylst])):
            position=camera_to_base(transform_mat, np.array([[x,y,z]])).reshape(-1)
            clusters.append(position)

        
            
        single_cluster = np.stack(clusters)
        cluster_list.append(single_cluster)
    
    cleaned_point_cloud_list = []
    for single_cluster in cluster_list:
        cleaned_point_cloud = clean_point_cloud(single_cluster,clean_scale)
        cleaned_point_cloud_list.append(cleaned_point_cloud)
    return cleaned_point_cloud_list

# %%
def project_YZ(cleaned_point_cloud, VIZ=False):
    # assume cleaned_point_cloud is the cleaned point cloud as a numpy array with shape (n_points, 3)

    # project points onto the XY plane by ignoring the Z coordinate
    projected_points = cleaned_point_cloud[:, [1,2]]

    # compute extent of points in XY plane
    min_x, min_y = np.min(projected_points, axis=0)
    max_x, max_y = np.max(projected_points, axis=0)

    # compute center of circle
    center_x, center_y = (min_x + max_x) / 2, (min_y + max_y) / 2

    # compute radius of circle
    radius = max((max_x - min_x), (max_y - min_y)) / 2

    # plot the circle and points
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.scatter(projected_points[:, 0], projected_points[:, 1], s=1)
    circle = plt.Circle((center_x, center_y), radius, fill=False)
    ax.add_artist(circle)

    if VIZ:
        edge = 0.02
        plt.xlim(center_x-radius-edge, center_x+radius+edge)
        plt.ylim(center_y-radius-edge, center_y+radius+edge)
        plt.axis('equal')
        plt.show()
    center_z = (min_y+max_y)/2
    return center_z






# %%
import numpy as np
import matplotlib.pyplot as plt

def project_XY(cleaned_point_cloud, edge = 0.01, SAVE=None):
    # assume cleaned_point_cloud is the cleaned point cloud as a numpy array with shape (n_points, 3)

    # project points onto the XY plane by ignoring the Z coordinate
    projected_points = cleaned_point_cloud[:, [1,0]]

    # compute extent of points in XY plane
    min_x, min_y = np.min(projected_points, axis=0)
    max_x, max_y = np.max(projected_points, axis=0)

    # compute center of circle
    center_x, center_y = (min_x + max_x) / 2, (min_y + max_y) / 2

    # compute radius of circle
    radius = (max_x - min_x)/ 2
    # plot the circle and points
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.scatter(projected_points[:, 0], projected_points[:, 1], s=1, c='k')
    # circle = plt.Circle((center_x, center_y), radius, fill=False)
    
    plt.xlim(center_x-radius-edge, center_x+radius+edge)
    plt.ylim(center_y-radius-edge, center_y+radius+edge)

    if SAVE is not None:
        plt.axis("off")
        plt.savefig(SAVE, pad_inches=0, bbox_inches='tight', transparent=True)
    plt.axis('on')

    info = dict(center_x = center_x, center_y = center_y, radius = radius, edge = edge)
    return ax,fig, info


# %%
def run_camera(perception_path, camera_p, rotation_mat, clean_scale=3):
    CAPTURE = True

    import subprocess
    import os

    if CAPTURE:
        if os.path.exists(f"{perception_path}/label_data.npy"):
            os.remove(f"{perception_path}/label_data.npy")

        if os.path.exists(f"{perception_path}/depth_data.npy"):
            os.remove(f"{perception_path}/depth_data.npy")

        subprocess.run([f"cd {perception_path}; ./experiments/scripts/ros_seg_rgbd_add_test_segmentation_realsense.sh $GPU_ID 0"], shell=True)
    
    label_pixel = np.load(f'{perception}/label_data.npy')
    depth_pixel = np.load(f'{perception}/depth_data.npy') 

    position_mat = Translation(*(camera_p.tolist()))
    transform_mat= HT_matrix(rotation_mat, position_mat)

    cleaned_point_cloud = cluster_info_2_cleaned_point_cloud(label_pixel, depth_pixel, transform_mat, clean_scale=clean_scale)

    return cleaned_point_cloud



# %%
def get_center_position(perception_path, camera_p, rotation_mat, clean_scale = 2, VIZ=False):
    cleaned_point_cloud_list = run_camera(perception_path, camera_p, rotation_mat, clean_scale)
    
    center_position_list = []
    radius_list = []
    for cleaned_point_cloud in cleaned_point_cloud_list:

        center_z = project_YZ(cleaned_point_cloud, VIZ=VIZ)
        center_x, center_y, radius = project_XY(cleaned_point_cloud, VIZ=VIZ)
        center_position_array = np.array([center_y,center_x, center_z])
        center_position_list.append(center_position_array)
        radius_list.append(radius)

    return center_position_list, radius_list


if __name__ == "__main__":
    capture_camera()

    label_pixel = np.load(f'{perception}/label_data.npy')
    depth_pixel = np.load(f'{perception}/depth_data.npy') 


    # Calibration
    rotation_x   = Rotation_X(-math.pi)
    rotation_y   = Rotation_Y(-math.pi/180*55)
    rotation_mat = np.dot(rotation_x, rotation_y)

    # camera_p = np.load("planned_traj/camera_p.npy")
    camera_p = np.array([ 0.33107886, -0.25199999,  1.08883276])

    position_mat = Translation(*(camera_p.tolist()))
    transform_mat= HT_matrix(rotation_mat, position_mat)
    cleaned_point_cloud = cluster_info_2_cleaned_point_cloud(label_pixel, depth_pixel, transform_mat, clean_scale=2)



    center_z = project_YZ(cleaned_point_cloud, VIZ=True)
    center_x, center_y, radius = project_XY(cleaned_point_cloud, VIZ=True)

    # center_position_array, radius = get_center_position(perception, camera_p, rotation_mat, clean_scale = 3, VIZ=True)

# %%

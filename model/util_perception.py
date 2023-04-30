import numpy as np
import pcl 
from sensor_msgs.msg import PointCloud2, PointField
import ctypes
import struct
import rospy 
import torch 
import torch.nn as nn 

def list_to_pcd(lista): #[x,y,z,R,G,B] 6dims 
    cloud = pcl.PointCloud_PointXYZRGB()
    cloud.from_list(lista)
    return cloud

def pcl_to_ros(pcl_array, frame_id="camera_link"):
    """ Converts a pcl PointXYZRGB to a ROS PointCloud2 message
    
        Args:
            pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud
            
        Returns:
            PointCloud2: A ROS point cloud
    """
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = frame_id

    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="rgb",
                            offset=16,
                            datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        s = struct.pack('>f', data[3])
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)

        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

    ros_msg.data = b"".join(buffer)

    return ros_msg


def get_cluster_points(depth_pixel, label_pixel, transform_mat):
    clustering_num=len(np.unique(label_pixel));target_cluster_center_point=None
    # Total cluster loop
    for cluster_idx in range(clustering_num):
        # Random color 
        # color = random_color_gen()
        cluster_xlst, cluster_ylst = np.where(label_pixel==cluster_idx+1)
        for idx, (x,y,z) in enumerate(zip(depth_pixel[2,cluster_xlst, cluster_ylst], \
                                            depth_pixel[0,cluster_xlst, cluster_ylst], \
                                            depth_pixel[1,cluster_xlst, cluster_ylst])):
            position=camera_to_base(transform_mat, np.array([[x,y,z]])).reshape(-1)
            # if idx==0: single_cluster=np.array([[position[0],position[1],position[2], \
            #                                      rgb_to_float([255, int(abs(position[1])*(256/0.3)),int(abs(position[1])*(256/0.3))])]])
            if idx==0: single_cluster=np.array([[position[0],position[1],position[2], \
                                                    rgb_to_float([110,110,110])]]) #Place: [255, 204, 204]
            else:
                if position[2]<0.72 or position[0]>1.3 or position[1]>0.45 or position[1]<-0.45: # Remove unnecessary part
                    pass 
                else: 
                    if position[0]>1.05: # Target object 
                        single_cluster = np.concatenate((single_cluster,np.array([[position[0],position[1],position[2],\
                                                                                    rgb_to_float([0,255,0])]])))
                    else: 
                        single_cluster = np.concatenate((single_cluster,np.array([[position[0],position[1],position[2],\
                                                                                    rgb_to_float([110,110,110])]])))
                    # else: 
                    #     single_cluster = np.concatenate((single_cluster,np.array([[position[0],position[1],position[2],\
                    #                                                                rgb_to_float([255,int(abs(position[1])*(256/0.3)),int(abs(position[1])*(256/0.3))])]])))
        if cluster_idx==0: total_clusters=single_cluster 
        else: total_clusters = np.concatenate((total_clusters, single_cluster))
        cen_x = np.average(single_cluster[:,0])+0.025; cen_y=np.average(single_cluster[:,1]); cen_z=np.average(single_cluster[:,2])
        print("Object Center", [cen_x,cen_y,cen_z])

        if cen_x >1.1: # Target object 
            cen_x+=0.01
            target_cluster_center_point=[cen_x,cen_y, cen_z]    
    return target_cluster_center_point, total_clusters

def interpolation(traj, interp_num=10):
    for idx in range(len(traj)):
        if (idx)== len(traj)-1:
            break 
        if idx==0: 
            interpoled_traj = np.linspace(traj[idx], traj[idx+1], num=interp_num)
            new_traj = interpoled_traj
        else: 
            interpoled_traj =  np.linspace(traj[idx], traj[idx+1], num=interp_num)
            new_traj = np.append(new_traj, interpoled_traj)
    return new_traj

def make_viz_lines(anchor_time, anchors, z=0.75, interp_num=1000):
    lines=[]
    interp_anchor_t = interpolation(anchor_time.reshape(-1), interp_num=interp_num)
    for anchor in anchors:
        line=[]
        interp_anchor = interpolation(anchor.reshape(-1), interp_num=interp_num)
        for x, y in zip(interp_anchor_t.reshape(-1), interp_anchor.reshape(-1)):
            line.append([x,y,z])
        lines.append(line)
    return lines

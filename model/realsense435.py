import rospy 
from sensor_msgs.msg import PointCloud2, Image
import time 
import numpy as np 
import message_filters
import cv_bridge 
import cv2 
from model.utils_projection import realworld_ortho_proejction,realworld_ortho_proejction_place

class RealsenseD435i():
    def __init__(self, mode="depth"):    
        self.tick = 0
        self.mode = mode 
        self.point_cloud = None 
        self.depth_image = None 
        self.rgb_image = None 
        self.cluster_pub = rospy.Publisher("/cluster_point", PointCloud2, queue_size=1)
        self.cluster_pub_np = rospy.Publisher("/cluster_point_np", PointCloud2, queue_size=1)
        self.cluster_pub_place = rospy.Publisher("/cluster_point_place", PointCloud2, queue_size=1)

        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        if mode=="pointcloud":
            self.point_cloud_sub = message_filters.Subscriber('/camera/depth/color/points', PointCloud2)
            self.ts = message_filters.TimeSynchronizer([self.point_cloud_sub, self.rgb_image_sub], 10)
        elif mode=="depth": 
            self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
            self.ts = message_filters.TimeSynchronizer([self.depth_image_sub, self.rgb_image_sub], 10)

        self.ts.registerCallback(self.callback)
        
        tic_temp = 0
        while self.tick<2:
            time.sleep(1e-3)
            tic_temp = tic_temp + 1
            if tic_temp > 5000:
                print ("[ERROR] CHECK REALSENSE435")
                break

    def callback(self, depth_msg, rgb_msg):
        self.tick = self.tick+1
        self.color_image = rgb_msg

        if self.mode == "pointcloud":
            self.point_cloud = depth_msg
        elif self.mode=="depth": 
            self.depth_image = depth_msg 
    
    def cluster_publishser(self, cluster_msg, mode=None): 
        if mode==None:
            self.cluster_pub.publish(cluster_msg)
        elif mode=='np':
            self.cluster_pub_np.publish(cluster_msg)
        elif mode=='place':
            self.cluster_pub_place.publish(cluster_msg)

def get_depth_img(depth_msg, mode='np'):
    bridge = cv_bridge.CvBridge()
    img_shape = (640, 480)
    try:
        cv_image_array = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        cv_image_array = np.array(cv_image_array, dtype = np.dtype('f8'))
        cv_image_array = cv2.resize(cv_image_array, img_shape, interpolation = cv2.INTER_CUBIC)
        cv_image_array = cv2.normalize(cv_image_array, cv_image_array, 0, 255, cv2.NORM_MINMAX)
        if mode=='np':
            cv_image_array = realworld_ortho_proejction(cv_image_array)
            cv_image_array = cv_image_array[:-33,:]
        elif mode=='place': 
            cv_image_array = realworld_ortho_proejction_place(cv_image_array)
        if np.isnan(np.max(cv_image_array)) or np.average(cv_image_array)<0.5:
            print('[Warning] Nan Detected!! Can Not Save IMAGE...')
            return False 
        else: 
            np.save("./data/npy/np_norm.npy", cv_image_array.astype(np.float32))
            cv2.imwrite('./data/png/np_norm.png', cv_image_array*255)
            return cv_image_array
    except cv_bridge.CvBridgeError as e:
        print(e) 



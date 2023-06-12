""" FOR ONROBOT RG2 """
from pymodbus.client.sync import ModbusTcpClient
""" FOR MODERN DRIVER """
import roslib; roslib.load_manifest('ur_driver')
import rospy    
import sys
sys.path.append("../../")
from model.gripper import openGrasp, closeGrasp

def main(): 
    graspclient = ModbusTcpClient('192.168.0.22') 
    closeGrasp(force=200, width=100, graspclient=graspclient)
    openGrasp(force=200, width=700, graspclient=graspclient)

main()
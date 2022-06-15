import rospy
import time
from pynput.keyboard import Key, Listener, KeyCode
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
import numpy as np
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState



def drive(array):
    jointCommand('',3,'Goal_Position', round(array[2]/1023 * 300 + 512),0.2)
    jointCommand('',4,'Goal_Position', round(array[3]/1023 * 300 + 512),0.2)
    jointCommand('',2,'Goal_Position', round(array[1]/1023 * 300 + 512),0.2)
    jointCommand('',1,'Goal_Position', round(array[0]/1023 * 300 + 512),0.2)

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))


if __name__ == '__main__':
    try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)
        ## Home
        jointCommand('', 1, 'Torque_Limit', 600, 0)
        jointCommand('', 2, 'Torque_Limit', 600, 0)
        jointCommand('', 3, 'Torque_Limit', 400, 0)
        jointCommand('', 4, 'Torque_Limit', 400, 0)
        # jointCommand('', 4, 'Goal_Position', 143 , 0.5)
        time.sleep(0.5)
        jointCommand('', 2, 'Goal_Position', 716, 0.5)
        time.sleep(0.5)
        jointCommand('', 1, 'Goal_Position', 512, 0.5)
        time.sleep(0.2)
        jointCommand('', 3, 'Goal_Position', 143, 1)
        time.sleep(0.5)
        jointCommand('', 4, 'Goal_Position', 800, 0.5)
        # jointCommand('', 1, 'Goal_Position', 512, 0.5)


        drive(q0);
        
    except rospy.ROSInterruptException:
        pass
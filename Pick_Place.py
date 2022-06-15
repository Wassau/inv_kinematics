import numpy as np
from matplotlib.pyplot import axes
import roboticstoolbox as rtb  # Robotics and numerical libraries
import rospy
from spatialmath import *
from spatialmath.base import *
from std_msgs.msg import * 
from dynamixel_workbench_msgs.srv import DynamixelCommand
import time
from PInvKin import PInvKin

motors_ids = [1,2,3,4,5]
L_L = [14.5, 10.63, 10.65, 8.97] # Length of links
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(alpha =np.pi/2, a=0, d=L_L[0], offset=0, ),#qlim=[-3*np.pi/4, 3*np.pi/4]),
        rtb.RevoluteDH(alpha =0, a=L_L[1], d=0, offset=np.pi/2, ),#qlim=[-3*np.pi/4, 3*np.pi/4]),
        rtb.RevoluteDH(alpha =0, a=L_L[2], d=0, offset=0, ),#qlim=[-3*np.pi/4, 3*np.pi/4]),
        rtb.RevoluteDH(alpha =0, a=0, d=0, offset=0, ),#qlim = [-3*np.pi/4, 3*np.pi/4]),
    ], 
    name="PhantomX",
    tool = SE3(np.array([[ 0,  0, 1, L_L[3]],[-1,  0, 0, 0], [ 0, -1, 0, 0], [ 0,  0, 0, 1]]))
)

print(robot)
q=(0, -np.pi/3, -np.pi/3, 0)
robot.plot(q,eeframe=True,jointaxes=False)
input()


def deg2raw(input_list: list = [0,0,0,0], min_deg: int = -150, max_deg: int = 150)->list:
    """
    Convert degrees array to 10 bit motor control value.
    """

    out_list = [0,0,0,0]
    for i in range(len(input_list)):
        out_list[i] = int( ((input_list[i] - min_deg)*1024)/(max_deg-min_deg) )
    return out_list




def jointCommand(command, id_num, addr_name, value, time):
    
    """
    Make a request to a the "dynamixel_command" ros service to modify a  
    parameter such as position or torque of the motor specified by the "id_num" 
    parameter.
    """
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))


def MTHK(x:float=4, y:float=4, z:float=5, phi:float=-np.pi)->np.array:
    """
    Generates the MTH matrix to execute inverse kinematics and returns
    an array with the joint position.
    """
    
    MTH = SE3(x,y,z)*SE3.Ry(phi)
    print(MTH)
    q = PInvKin(MTH.data[0], L_L)
    return q[1]

def setFullPostion(q:np.array = np.array([0, 0, 0, 0]) )->None:
    """
    Set all the joints of the robot to the position specified by the input array
    """
    robot.plot(q)
    # q_raw=deg2raw(np.degrees(q))
    # print(q_raw)
    # jointCommand('', motors_ids[0], 'Goal_Position', q_raw[0], 0)
    # jointCommand('', motors_ids[1], 'Goal_Position', q_raw[1], 0)
    # jointCommand('', motors_ids[2], 'Goal_Position', q_raw[2], 0)
    # jointCommand('', motors_ids[3], 'Goal_Position', q_raw[3], 0)

def setGripperPosition(servoValue:int=0)->None:
    """
    Set the gripper position to the specified 0 to 1023 value
    """

    jointCommand('', motors_ids[4], 'Goal_Position', servoValue, 0)

def main():
    
    # Movement parameters 
    home_x_position = 10
    home_y_position =  0

    home_to_base_x_postion = 10
    home_to_base_y_postion = -12

    base_x_position = 0
    base_y_position =-12

    home_to_load_x_postion = 10
    home_to_load_y_postion = 12

    load_x_position = 0
    load_y_position = 12

    free_movement_safe_height = 12
    base_gripper_height = 4
    load_gripper_height = 4.5
    load_over_base_height = 7

    # Gripper values
    base_gripper_closed_value = 348
    load_gripper_closed_value = 348
    gripper_open_value = 512


    # Number of steps of each movement phase
    home_to_base_x_steps = 10
    home_to_base_y_steps = 10
    home_to_load_x_steps = 10
    home_to_load_y_steps = 10

    pick_up_base_down_steps = 20
    pick_up_base_up_steps = 20
    place_base_down_steps = 20
    home_up_steps = 20
    pick_uL_Load_down_steps = 20
    pick_uL_Load_up_steps = 20
    place_load_down_steps = 20

    # Trajectory generation:


    q_vector1 = np.zeros((1 + pick_up_base_down_steps+home_to_base_y_steps+home_to_base_x_steps,4))
    q_vector2 = np.zeros((pick_up_base_up_steps + place_base_down_steps+home_to_base_x_steps+home_to_base_y_steps,4))
    q_vector3 = np.zeros((home_up_steps + pick_uL_Load_down_steps+home_to_load_y_steps+home_to_load_x_steps,4))
    q_vector4 = np.zeros((pick_uL_Load_up_steps +home_to_load_x_steps+home_to_load_y_steps+ place_load_down_steps,4))

    # 1-homming
    q_vector1[0] = MTHK(x = home_x_position,
                        y = home_y_position,
                        z = free_movement_safe_height
                   )
    robot.plot(q_vector1[0])
    input()
    # 1-home->base_position

    for i in range(home_to_base_y_steps):
        print ('vector')
        q_vector1[i+1] = MTHK(x=home_x_position,
                              y=home_y_position+
                              (((home_to_base_y_postion-home_y_position)
                              *(i+1))/home_to_base_y_steps),
                              z = free_movement_safe_height
                         ) 
                         
                         
    
    for i in range(home_to_base_x_steps):
        q_vector1[i+home_to_base_y_steps+1] = MTHK(x=home_to_base_x_postion+
                                                 (((base_x_position-home_to_base_x_postion)
                                                 *(i+1))/home_to_base_x_steps),
                                                 y = home_to_base_y_postion,
                                                 z = free_movement_safe_height
                                            )
    # 2 pick-up-base_down

    for i in range(pick_up_base_down_steps):
        q_vector1[i+home_to_base_y_steps + home_to_base_x_steps+1] = MTHK(x=base_x_position,
                                                                        y=base_y_position,
                                                                        z=free_movement_safe_height-
                                                                        (((free_movement_safe_height-base_gripper_height)
                                                                        *(i+1))/pick_up_base_down_steps)
                                                                    )
    # 3 pick-up-base_up

    for i in range(pick_up_base_up_steps):
        q_vector2[i] = MTHK(x=base_x_position,
                            y=base_y_position,
                            z=base_gripper_height+
                            (((free_movement_safe_height-base_gripper_height)
                            *(i+1))/pick_up_base_up_steps)
                       )

    # 4-base_position->home

    for i in range(home_to_base_x_steps):
        q_vector2[i+pick_up_base_up_steps] = MTHK(x=base_x_position+
                                                 (((home_to_base_x_postion-base_x_position)
                                                 *(i+1))/home_to_base_x_steps),
                                                 y = base_y_position,
                                                 z = free_movement_safe_height
                                            )
    for i in range(home_to_base_y_steps):
        q_vector2[pick_up_base_up_steps+home_to_base_x_steps+i] = MTHK(x=home_to_base_x_postion,
                                                                       y=home_to_base_y_postion+
                                                                       (((home_y_position-home_to_base_y_postion)
                                                                       *(i+1))/home_to_base_y_steps),
                                                                       z = free_movement_safe_height
                                                                   ) 
    # 5-place-base_down

    for i in range(place_base_down_steps):
        q_vector2[i+pick_up_base_up_steps+home_to_base_x_steps+home_to_base_x_steps] = MTHK(x=home_x_position,
                                                                                            y=home_y_position,
                                                                                            z=free_movement_safe_height-
                                                                                            (((free_movement_safe_height-base_gripper_height)
                                                                                            *(i+1))/place_base_down_steps)
                                                                                       )
    # 6-home-up

    for i in range(home_up_steps):
        q_vector3[i] = MTHK(x=home_x_position,
                            y=home_y_position,
                            z=base_gripper_height+
                            (((free_movement_safe_height-base_gripper_height)
                            *(i+1))/home_up_steps)
                       )

    # 7-home->load

    for i in range(home_to_load_y_steps):
        q_vector3[home_up_steps+i] = MTHK(x=home_x_position,
                                          y=home_y_position+
                                          (((home_to_load_y_postion-home_y_position)
                                          *(i+1))/home_to_load_y_steps),
                                          z = free_movement_safe_height
                                      )
    for i in range(home_to_base_x_steps):
        q_vector3[i+home_to_load_y_steps+home_up_steps] = MTHK(x=home_to_load_x_postion+
                                                               (((load_x_position-home_to_load_x_postion)
                                                               *(i+1))/home_to_base_x_steps),
                                                               y = home_to_load_y_postion,
                                                               z = free_movement_safe_height
                                                           )

    # 8-pick-up-load_down
    for i in range(pick_uL_Load_down_steps):
        q_vector3[i+home_to_load_y_steps+home_to_base_x_steps+home_up_steps] = MTHK(x=load_x_position,
                                                                                    y=load_y_position,
                                                                                    z=free_movement_safe_height-
                                                                                    (((free_movement_safe_height-load_gripper_height)
                                                                                    *(i+1))/pick_uL_Load_down_steps)
                                                                               )

    #9-pick-up-load_up
    for i in range(pick_uL_Load_up_steps):
        q_vector4[i] = MTHK(x=load_x_position,
                            y=load_y_position,
                            z=load_gripper_height+
                            (((free_movement_safe_height-load_gripper_height)
                            *(i+1))/pick_uL_Load_up_steps)
                       )
    
    #10 load->home

    for i in range(home_to_load_x_steps):
        q_vector4[i+pick_uL_Load_up_steps] = MTHK(x=load_x_position+
                                                 (((home_to_load_x_postion-load_x_position)
                                                 *(i+1))/home_to_load_x_steps),
                                                 y = load_y_position,
                                                 z = free_movement_safe_height
                                            )
    for i in range(home_to_load_y_steps):
        q_vector4[pick_uL_Load_up_steps+home_to_load_x_steps+i] = MTHK(x=home_to_load_x_postion,
                                                                       y=home_to_load_y_postion+
                                                                       (((home_y_position-home_to_load_y_postion)
                                                                       *(i+1))/home_to_load_y_steps),
                                                                       z = free_movement_safe_height
                                                                   ) 


    #11 place-load-down

    for i in range(place_load_down_steps):
        q_vector4[i+pick_up_base_up_steps+home_to_load_x_steps+home_to_load_y_steps] = MTHK(x=home_x_position,
                                                                                              y=home_y_position,
                                                                                              z=free_movement_safe_height-
                                                                                              (((free_movement_safe_height-load_over_base_height)
                                                                                              *(i+1))/place_load_down_steps)
                                                                                        )


    q_vector = np.concatenate((q_vector1,q_vector2,q_vector3,q_vector4),axis=0)
    
    # Motors configuration

    jointCommand('', motors_ids[0], 'Torque_Limit', 300, 0)
    jointCommand('', motors_ids[1], 'Torque_Limit', 500, 0)
    jointCommand('', motors_ids[2], 'Torque_Limit', 300, 0)
    jointCommand('', motors_ids[3], 'Torque_Limit', 300, 0)
    jointCommand('', motors_ids[4], 'Torque_Limit', 300, 0)
    
    jointCommand('', motors_ids[0], 'Torque_Enable', 1, 0)
    jointCommand('', motors_ids[1], 'Torque_Enable', 1, 0)
    jointCommand('', motors_ids[2], 'Torque_Enable', 1, 0)
    jointCommand('', motors_ids[3], 'Torque_Enable', 1, 0)
    jointCommand('', motors_ids[4], 'Torque_Enable', 1, 0)

    # Initial gripper open position
    setGripperPosition(gripper_open_value)

    # Movement
    time.sleep(1)
    for i in q_vector1:
        setFullPostion(i)    
    setGripperPosition(base_gripper_closed_value)

    time.sleep(0.8)         # Stabnilization delay
    for i in q_vector2:
        setFullPostion(i)
    setGripperPosition(gripper_open_value)
    time.sleep(0.8)
    for i in q_vector3:
        setFullPostion(i)
    setGripperPosition(load_gripper_closed_value)
    time.sleep(0.8)
    for i in q_vector4:
        setFullPostion(i)
    setGripperPosition(gripper_open_value)
    time.sleep(0.5)
    
        
def main1():
    setFullPostion(np.array([0,0,0,np.pi]))
    
    time.sleep(10)

def test():
    robot.plot([0,0,0,0],jointaxes = True, backend="pyplot")
    pos = [13,13,4]
    for i in range(100):
        setFullPostion(np.array([pos[0], pos[1], pos[2],-np.pi]))
        time.sleep(0.1)
        pos[0]-=0.1
        pos[1]-=0.1
        #pos[2]+=0.1
    
    time.sleep(20)
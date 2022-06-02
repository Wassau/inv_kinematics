import rospy
import time
from pynput.keyboard import Key, Listener, KeyCode
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
import numpy as np
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from ctypes import sizeof
from unicodedata import name
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
# import ikunD

# def drive(array):
#     jointCommand('',3,'Goal_Position', round(-array[2]/1023 * 300 + 512),0.2)
#     print(round(array[2]/1023 * 300 + 512))
#     print(type(round(array[2]/1023 * 300 + 512)))
#     time.sleep(0.5)
#     jointCommand('',4,'Goal_Position', round(-array[3]/1023 * 300 + 512),0.2)
#     print(round(array[3]/1023 * 300 + 512))
#     time.sleep(0.5)
#     jointCommand('',2,'Goal_Position', round(array[1]/1023 * 300 + 512),0.2)
#     print(round(array[1]/1023 * 300 + 512))
#     time.sleep(0.5)
#     jointCommand('',1,'Goal_Position', round(array[0]/1023 * 300 + 512),0.2)
    
def jointCommand(command, id_num, addr_name, value, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command, id_num, addr_name, value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))


Pincher = rtb.DHRobot(
    [
        rtb.RevoluteDH(alpha= np.pi/2, d= 4.45),
        rtb.RevoluteDH(a=10.49,offset=np.pi/2),
        rtb.RevoluteDH(a=10.7),
        rtb.RevoluteDH(alpha=np.pi/2)

    ], name="Pincher"
)

print(Pincher)
Pincher.plot([0, -np.pi/3, np.pi/3, -np.pi/2])
q0=Pincher.fkine( [0, -np.pi/3, np.pi/3, -np.pi/2])
print(q0)
print('Home')
input()
## Encima pincho
T0 = SE3( 4, -4, 11 ) * SE3.OA([0, 1, 0], [0, 0, -1]) 
sol0 = Pincher.ikine_LM(T0)  
print('Pose1')
print(np.rad2deg(sol0.q))
qf = Pincher.fkine(np.round(sol0.q))
qt = rtb.jtraj([0, -np.pi/3, np.pi/3, -np.pi/2], sol0.q, 10)
print('titulo:' )
print( qt.q)
input()
t2 = qt.q[9]
qtdeg = np.rad2deg(np.round(qt.q))
qtdeg = qtdeg.astype(int)
print(qtdeg)
#recogiendo pincho

T1 = SE3( 4, -4, 1.5 ) * SE3.OA([0, 1, 0], [0, 0, -1])  
sol1 = Pincher.ikine_LM(T1,q0= [0, -np.pi/3, np.pi/3, -np.pi/2])        # solve IK
q_pickup = np.round(sol1.q,5)
qt = Pincher.fkine(q_pickup)
print(np.rad2deg(q_pickup))
# Pincher.plot(q_pickup)
print('titulo:' )
print(t2)
qt = rtb.jtraj(t2, sol1.q, 10)
config = sol0.q.astype(int)
print( qt.q)
input()
t2 = qt.q[9]

## Home2 
T2 = SE3( 6.77, 0, 21.83 ) * SE3.OA([1, 0, 0], [0, 0, -1]) 
sol2 = Pincher.ikine_LM(T2)
print(np.rad2deg(sol2.q))
# Pincher.plot(sol2.q)
print('titulo:' )
qt = rtb.jtraj(t2, sol2.q, 10)
config = sol0.q.astype(int)
print( qt.q)
input()
t2 = qt.q[9]

## Suelta pincho

T3 = SE3(  6.77, 0, .83 ) * SE3.OA([0, 1, 0], [0, 0, -1]) 
sol3 = Pincher.ikine_LM(T3, q0=[0, -np.pi/3, np.pi/3, -np.pi/2])  
# Pincher.plot(sol0.q)
print('titulo:' )
qt = rtb.jtraj(t2, sol3.q, 10)
print( qt.q)
t2 = qt.q[9]

## Home2
T4 = SE3( 6.77, 0, 21.83 ) * SE3.OA([0, 1, 0], [0, 0, -1]) #home
sol4 = Pincher.ikine_LM(T4)
print('titulo:' )
qt = rtb.jtraj(t2, sol4.q, 10)
print( qt.q)
t2 = qt.q[9]
# Encima de roscon
T5 = SE3( 4, 4, 11 ) * SE3.OA([0, 1, 0], [0, 0, -1]) #Home rotada a la derecha
sol5 = Pincher.ikine_LM(T5)  
print('titulo:' )
qt = rtb.jtraj(t2, sol5.q, 10)
print( qt.q)
t2 = qt.q[9]

## recoge roscon

T6 = SE3( 4, 4, 3 ) * SE3.OA([0, 1, 0], [0, 0, -1]) #Home rotada a la derecha
sol6 = Pincher.ikine_LM(T6)  
print('titulo:' )
qt = rtb.jtraj(t2, sol6.q, 10)
print( qt.q)
t2 = qt.q[9]

if __name__ == '__main__':
    try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)
        ## Home
        jointCommand('', 1, 'Torque_Limit', 600, 0)
        jointCommand('', 2, 'Torque_Limit', 600, 0)
        jointCommand('', 3, 'Torque_Limit', 600, 0)
        jointCommand('', 4, 'Torque_Limit', 600, 0)
        # jointCommand('', 4, 'Goal_Position', 143 , 0.5)
        time.sleep(0.5)
        jointCommand('', 2, 'Goal_Position', 306, 0.5)
        time.sleep(0.5)
        jointCommand('', 1, 'Goal_Position', 512, 0.5)
        time.sleep(0.2)
        jointCommand('', 3, 'Goal_Position', 306, 1)
        time.sleep(0.5)
        jointCommand('', 4, 'Goal_Position', 512, 0.5)
        # jointCommand('', 1, 'Goal_Position', 512, 0.5)
       
        T0 = SE3( 5, -5, 15 ) * SE3.OA([0, -1, 0], [0, 0, -1]) 
        sol0 = Pincher.ikine_LM(T0)  
        print('Pose1')
        input()
        print(np.rad2deg(sol0.q))
        qf = Pincher.fkine(np.round(sol0.q))
        qt = rtb.jtraj([0, -np.pi/3, -np.pi/3, 0], sol0.q, 10)
        print('titulo:' )
        print( qt.q)
        input()
        t2 = qt.q[9]
        qtdeg = np.rad2deg(np.round(qt.q))
        qtdeg = qtdeg.astype(int)
        print(qtdeg)
        for i in range(10):
            for j in range(4):
                print(qtdeg[i][j]/300 * 1023 + 512)
                jointCommand('', j+1, 'Goal_Position', round(qtdeg[i][j]/300 * 1023 + 512), 0.5)

                
        #recogiendo pincho
        input()
        T1 = SE3( 5, -5, 8 ) * SE3.OA([0, 1, 0], [0, 0, -1])  
        sol1 = Pincher.ikine_LM(T1,q0=t2)        # solve IK
        q_pickup = np.round(sol1.q,5)
        qt = Pincher.fkine(q_pickup)
        print(np.rad2deg(q_pickup))
        # Pincher.plot(q_pickup)
        print('titulo:' )
        print(t2)
        qt = rtb.jtraj(t2, sol1.q, 10)
        qtdeg = np.rad2deg(np.round(qt.q))
        qtdeg = qtdeg.astype(int)
        print(qtdeg)
        for i in range(10):
            for j in range(4):
                print(qtdeg[i][j]/300 * 1023 + 512)
                jointCommand('', j+1, 'Goal_Position', round(qtdeg[i][j]/300 * 1023 + 512), 0.5)


        
            



        
    except rospy.ROSInterruptException:
        pass
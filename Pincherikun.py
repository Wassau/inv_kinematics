from ctypes import sizeof
from json import tool
from unicodedata import name
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from spatialmath.base import *
# import ikunD


def drive(array):
    jointCommand('',3,'Goal_Position', round(array[2]/1023 * 300 + 512),0.2)
    jointCommand('',4,'Goal_Position', round(array[3]/1023 * 300 + 512),0.2)
    jointCommand('',2,'Goal_Position', round(array[1]/1023 * 300 + 512),0.2)
    jointCommand('',1,'Goal_Position', round(array[0]/1023 * 300 + 512),0.2)

Pincher = rtb.DHRobot(
    [
        rtb.RevoluteDH(alpha= np.pi/2, d= 14.45),
        rtb.RevoluteDH(a=10.49,offset=np.pi/2),
        rtb.RevoluteDH(a=10.7),
        rtb.RevoluteDH(alpha=np.pi/2)

    ], name="Pincher",
        # tool =SE3(transl(0 , 0 , 8))
)

print(Pincher)
Pincher.plot([0, -np.pi/3, -np.pi/3, -np.pi/2])
q0=Pincher.fkine( [0, -np.pi/3, -np.pi/3, 0])
print(q0)
print('Home')
input()
## Encima pincho
T0 = SE3( 4, -4, 11 ) * SE3.OA([0, 1, 0], [0, 0, -1]) 
sol0 = Pincher.ikine_LM(T0)  
print('Pose1')
print(np.rad2deg(sol0.q))
qf = Pincher.fkine(np.round(sol0.q))
qt = rtb.jtraj([0, -np.pi/3, -np.pi/3, 0], sol0.q, 10)
print('titulo:' )
qtdeg = np.rad2deg(np.round(qt.q))
print(qtdeg[1])
input()
for i in range(len(qt)):
    # q = Pincher.ikine_LM(qt[i],q0= config )
    # config = np.rad2deg(np.round(q.q))
    t2=qt.q[i]
    print(np.rad2deg(qt.q[i]))
    Pincher.plot(qt.q[i])
#     print (qt[i])
# Pincher.plot(sol0.q)
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

for i in range(len(qt)):
    # q = Pincher.ikine_LM(qt[i],q0= config )
    # config = np.rad2deg(np.round(q.q))
    t2=qt.q[i]
    print(np.rad2deg(qt.q[i]))
    Pincher.plot(qt.q[i])
#     print (qt[i])
# Pincher.plot(sol0.q)

## Home2 
T2 = SE3( 6.77, 0, 21.83 ) * SE3.OA([1, 0, 0], [0, 0, -1]) 
sol2 = Pincher.ikine_LM(T2)
print(np.rad2deg(sol2.q))
# Pincher.plot(sol2.q)
print('titulo:' )
qt = rtb.jtraj(t2, sol2.q, 10)
config = sol0.q.astype(int)
for i in range(len(qt)):
    # q = Pincher.ikine_LM(qt[i],q0= config )
    # config = np.rad2deg(np.round(q.q))
    t2=qt.q[i]
    print(np.rad2deg(qt.q[i]))
    Pincher.plot(qt.q[i])
#     print (qt[i])
# Pincher.plot(sol0.q)


## Suelta pincho

T3 = SE3(  6.77, 0, .83 ) * SE3.OA([0, 1, 0], [0, 0, -1]) 
sol3 = Pincher.ikine_LM(T3, q0=[0, -np.pi/3, np.pi/3, -np.pi/2])  
# Pincher.plot(sol0.q)
print('titulo:' )
qt = rtb.jtraj(t2, sol3.q, 10)
print( qt.q)
for i in range(len(qt)):
    # q = Pincher.ikine_LM(qt[i],q0= config )
    # config = np.rad2deg(np.round(q.q))
    t2=qt.q[i]
    print(np.rad2deg(qt.q[i]))
    Pincher.plot(qt.q[i])
#     print (qt[i])
# Pincher.plot(sol0.q)

## Home2
T4 = SE3( 6.77, 0, 21.83 ) * SE3.OA([0, 1, 0], [0, 0, -1]) #home
sol4 = Pincher.ikine_LM(T4)
print('titulo:' )
qt = rtb.jtraj(t2, sol4.q, 10)
print( qt.q)
for i in range(len(qt)):
    # q = Pincher.ikine_LM(qt[i],q0= config )
    # config = np.rad2deg(np.round(q.q))
    t2=qt.q[i]
    print(np.rad2deg(qt.q[i]))
    Pincher.plot(qt.q[i])
#     print (qt[i])
# Pincher.plot(sol0.q)

# Encima de roscon
T5 = SE3( 4, 4, 11 ) * SE3.OA([0, 1, 0], [0, 0, -1]) #Home rotada a la derecha
sol5 = Pincher.ikine_LM(T5)  
print('titulo:' )
qt = rtb.jtraj(t2, sol5.q, 10)
print( qt.q)
for i in range(len(qt)):
    # q = Pincher.ikine_LM(qt[i],q0= config )
    # config = np.rad2deg(np.round(q.q))
    t2=qt.q[i]
    print(np.rad2deg(qt.q[i]))
    Pincher.plot(qt.q[i])
#     print (qt[i])
# Pincher.plot(sol0.q)

## recoge roscon

T6 = SE3( 4, 4, 3 ) * SE3.OA([0, 1, 0], [0, 0, -1]) #Home rotada a la derecha
sol6 = Pincher.ikine_LM(T6)  
print('titulo:' )
qt = rtb.jtraj(t2, sol6.q, 10)
print( qt.q)
for i in range(len(qt)):
    # q = Pincher.ikine_LM(qt[i],q0= config )
    # config = np.rad2deg(np.round(q.q))
    t2=qt.q[i]
    print(np.rad2deg(qt.q[i]))
    Pincher.plot(qt.q[i])
#     print (qt[i])
# Pincher.plot(sol0.q)
T7 = SE3( 4, 4, 11 ) * SE3.OA([-1, 0, 0], [0, 0, -1]) #Home rotada a la derecha
sol7 = Pincher.ikine_LM(T7,q0=t2)  
print('titulo:' )
qt = rtb.jtraj(t2, sol7.q, 10)
print( qt.q)
for i in range(len(qt)):
    # q = Pincher.ikine_LM(qt[i],q0= config )
    # config = np.rad2deg(np.round(q.q))
    t2=qt.q[i]
    print(np.rad2deg(qt.q[i]))
    Pincher.plot(qt.q[i])
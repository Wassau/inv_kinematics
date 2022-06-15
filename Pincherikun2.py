from ctypes import sizeof
from json import tool
from operator import pos
from typing import Any
from unicodedata import name
from xml.etree.ElementTree import PI
import roboticstoolbox as rtb
from spatialmath import *
import numpy as np
from spatialmath.base import *
from PInvKin import PInvKin

griper_closed = 348
load_gripper_closed_value = 348
gripper_open_value = 512


L_L = [0.45, 1.063, 1.065, 0.897]
Pincher = rtb.DHRobot(
    [
        rtb.RevoluteDH(alpha =np.pi/2, a=0, d=L_L[0], offset=0, ),#qlim=[-3*np.pi/4, 3*np.pi/4]),
        rtb.RevoluteDH(alpha =0, a=L_L[1], d=0, offset=np.pi/2, ),#qlim=[-3*np.pi/4, 3*np.pi/4]),
        rtb.RevoluteDH(alpha =0, a=L_L[2], d=0, offset=0, ),#qlim=[-3*np.pi/4, 3*np.pi/4])
        rtb.RevoluteDH(alpha =0, a=0, d=0, offset=0, ),#qlim = [-3*np.pi/4, 3*np.pi/4]),
    ], 
    name="PhantomX",
    tool =SE3(transl(0 , 0 , L_L[3])@ troty(np.pi / 3))
)

qhome = [0, -np.pi/3, -np.pi/3, 0 ]
print('Home')
q0=Pincher.fkine( qhome )
print(q0)
Pincher.plot(qhome,eeframe= True, jointaxes = False, )
# Pincher.teach([0, -np.pi/3, -np.pi/3, 0],eeframe=True, jointaxes=False, name=True)
input()

## Encima pincho
T1 = SE3(0.4, -0.4, 0.5 ) * SE3.Ry(-np.pi)
ikun1 = PInvKin(T1.data[0], L_L)
print(np.rad2deg(ikun1[1]))

qf = rtb.jtraj(qhome, ikun1[1], 10)
qtdeg = np.rad2deg(np.round(qf.q))
for i in range(len(qf)):
    t2=qf.q[i]
    print(np.rad2deg(qf.q[i]))
    Pincher.plot(qf.q[i],jointaxes=False)

Pincher.plot(t2,jointaxes=False)
print('qf')
print(qf.q)
input()


## Recogiendo Pincho
T2 = SE3( 0.4, -0.4, 0.015 ) * SE3.Ry(-np.pi)
ikun2 = PInvKin(T2.data[0], L_L)
print(np.rad2deg(ikun2[1]))
Pincher.plot(ikun2[1],jointaxes=False)
qf = rtb.jtraj(ikun1[1], ikun2[1], 10)
qtdeg = np.rad2deg(np.round(qf.q))
for i in range(len(qf)):
    t2=qf.q[i]
    print(np.rad2deg(qf.q[i]))
    Pincher.plot(qf.q[i],jointaxes=False)
    
print('qf')
print(qf.q)
input()



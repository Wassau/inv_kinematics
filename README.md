# inv_kinematics

This repository explains and shows the development of the thrid Lab, which main objective is to learn about inverse kinematics and 

```python
import rospy
import time
from pynput.keyboard import Key, Listener, KeyCode
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
import numpy as np
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
  
```
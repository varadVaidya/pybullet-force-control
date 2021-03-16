import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb

## set the style for matplotli
robot = Manipulator()
trajtime = 5 # sec

simTime = 10 # sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)

initjointAngles = np.array([0,0,0,0,0,0])
finaljointAngles = np.array([1,1,1,1,1,1])

robot.getJointsTrajectory(initjointAngles,finaljointAngles,trajtime,time)

print(robot.TrajectoryPlan.jointAngleTraj[0])
print("joint angles are: ",robot.TrajectoryPlan.jointAngleTraj[:,1])

print("joint angles are: ",robot.TrajectoryPlan.jointAngleTraj.shape)
print(robot.TrajectoryPlan.jointVelTraj[0])

'''
the joint angles provided are : [0.00000000e+00 2.08564959e-06 8.33795716e-06 ... 9.99991662e-01
 9.99997914e-01 1.00000000e+00]
 
 when added with the full simulation time test
 the results were:
 [0.00000000e+00 2.08564959e-06 8.33795716e-06 ... 1.00000000e+00
 1.00000000e+00 1.00000000e+00]
[0.         0.001      0.00199833 ... 0.         0.         0.        ]

 
'''
import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb

robot = Manipulator()

initjointAngles = [0,-1.57,1.57,-1.57,-1.57,-1.57]
robot.setJointAngles(initjointAngles)
finalJointAngles = [0,0,1.4,0.,0.,-1]

# sim time parameters
simTime = 300 # sec
trajTime = 20 #sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)

K_Matrix = np.diag(6*[3])
D_Matrix = np.diag(6*[2])

robot.turnOFFActuators()
robot.turnOFFInternalDamping()

robot.getJointsTrajectory(initjointAngles,finalJointAngles,trajTime,time)


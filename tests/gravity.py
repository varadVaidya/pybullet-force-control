import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb
from time import sleep
## set the style for matplotli
robot = Manipulator()

jointAngles = [1,-1.5,1.5,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

FREE_FALL = False

robot.turnOFFActuators()
## sets the arm in free fall as no torques is applied on the joints.
if FREE_FALL:
    torque = 6*[0]
    
    while True:
        
        pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL,forces = torque)
        sleep(0.01)
        pb.stepSimulation()
        
if not FREE_FALL:
    
    while True:
        
        robot.getJointInfo()
        robot.getDyanamicMatrices()
        
        torque = robot.DynamicMatrices.gravityVector
        
        pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL,forces = torque)
        sleep(0.01)
        pb.stepSimulation()

#### the robot is able to hold itself against gravity.
### hence the ggravity vector calculation is correct.
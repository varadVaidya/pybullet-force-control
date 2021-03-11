import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb

robot = Manipulator()

jointAngles = [-1,-2.7,2.4,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

desEndEffector = np.array([0.7,0.2,0.5,0.2,1.2,0])
desEndEffectorVel = np.array([0,0,0,0,0,0])

Kp = np.diag([.1,.1,.1,0.1,0.1,0.1])
Kd = np.diag([0.1,0.1,0.1,0.1,0.1,0.1])

while True:
    
    ##finding the state of the robot
    robot.getJointInfo()
    robot.getForwardKinematics()
    robot.calculateJacobian()
    
    endEffectorVel = robot.Jacobian.analyticJacobian.dot(robot.jointState.jointVelocities)
    
    positionError = desEndEffector - np.array(robot.forwardKinematics.linkState)
    velocityError = desEndEffectorVel - endEffectorVel
    
    commandVelocity = Kp.dot(positionError) + Kd.dot(velocityError)
    commandJointVelocity = robot.Jacobian.analyticJacobianInv.dot(commandVelocity)
    
    pb.setJointMotorControlArray(robot.armID , robot.controlJoints ,pb.VELOCITY_CONTROL,targetVelocities=commandJointVelocity)
    pb.stepSimulation()
    
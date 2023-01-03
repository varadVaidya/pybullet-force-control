import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb
#from ..manipulator.manipulator import Manipulator

## set the style for matplotli
robot = Manipulator()

jointAngles = [0,-1.57,1.57,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

# sim time parameters
simTime = 300 # sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)

robot.getJointInfo()
robot.getForwardKinematics()
desEndEffector = robot.forwardKinematics.linkState
desPos = desEndEffector[0:3]
desQuatXYZ = np.array(pb.getQuaternionFromEuler(desEndEffector[3:6])[0:3])
desQuatW = np.array(pb.getQuaternionFromEuler(desEndEffector[3:6])[3])
desEndEffectorVel = np.array([0,0,0,0,0,0])


Kp = np.diag(3*[1])
Kd = np.diag(3*[1])

for i in range(len(time)):
    
    ##finding the state of the robot
    robot.getJointInfo()
    robot.getForwardKinematics()
    robot.calculateJacobian()
    
    endEffectorVel = robot.Jacobian.analyticJacobian.dot(robot.jointState.jointVelocities)
    
    n1,q1,n2,q2 = robot.forwardKinematics.linkOrientationQuaternionW,robot.forwardKinematics.linkOrientationQuaternionXYZ,desQuatW,desQuatXYZ
    
    deltaN = n1*n2 + q1.T.dot(q2)
    deltaQ = n1*q2 - n2*q1 - np.cross(q1,q2)
    
    positionError = np.array(robot.forwardKinematics.linkPosition) - np.array(desPos)
    orientationError = deltaQ
    
    
    controlVelocity = np.hstack((
        desEndEffectorVel[0:3] - Kp.dot(positionError),
        desEndEffectorVel[3:6] - Kd.dot(orientationError)
    ))
    
    jointVelocity = robot.Jacobian.geometricJacobianInv.dot(controlVelocity)
    
    pb.setJointMotorControlArray(robot.armID , robot.controlJoints ,pb.VELOCITY_CONTROL,targetVelocities=jointVelocity)
    pb.stepSimulation()
    
    
    
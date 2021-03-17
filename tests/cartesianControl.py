import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb


## set the style for matplotli
robot = Manipulator()
pb.setRealTimeSimulation(False)
jointAngles = [1.57,-1.7,2.4,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

robot.turnOFFActuators()
robot.turnOFFInternalDamping()
desEndEffector = np.array([0.7,0.2,1.5,0.2,1.2,0.5])
desEndEffectorVel = np.array([0,0,0,0,0,0])

Kp = np.diag([9,9,9,0.3,.3,.3])
Kd = np.diag([9,9,9,0,0,0])


# sim time parameters
simTime = 320 # sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)

for i in range(len(time)):


   
    robot.getJointInfo()
    robot.getForwardKinematics()
    robot.calculateJacobian()
    robot.getDyanamicMatrices()
    
    
    endEffectorVel = robot.Jacobian.analyticJacobian.dot(robot.jointState.jointVelocities)
    
    positionError = desEndEffector - np.array(robot.forwardKinematics.linkState)
    
    robot.plotError.append(positionError)
    
    velocityError = desEndEffectorVel - endEffectorVel
    
    commanadedForce = Kp.dot(positionError) + Kd.dot(velocityError)
    
    commanadedJointTorque = robot.Jacobian.analyticJacobian.T.dot(commanadedForce) + robot.DynamicMatrices.gravityVector
    
    
    pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL, forces = commanadedJointTorque)
    pb.stepSimulation()
    
plotError = np.array(robot.plotError)
print("i reached here")
robot.plotValues(plotError,time)
import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb



robot = Manipulator()
pb.setRealTimeSimulation(False)
jointAngles = [1.57,-1.7,2.4,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)
jointAngles = np.array(jointAngles)

robot.turnOFFActuators()
robot.turnOFFInternalDamping()

desJointAngles = [0,-1.57,1.57,-1.57,-1.57,-1.57]

# sim time parameters
simTime = 40 # sec
trajTime = 15 #sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)

robot.getJointsTrajectory(jointAngles,desJointAngles,trajTime,time)
desEndEffectorVel = np.array([0,0,0,0,0,0])

Kp = np.diag([3,3,3,1,1,1])
Kd = np.diag([2,2,2,0.1,0.1,0.1])

for i in range(len(time)):
    robot.getJointInfo()
    robot.getForwardKinematics()
    robot.calculateJacobian()
    robot.getDyanamicMatrices()
    
    jointPositonError = robot.TrajectoryPlan.jointAngleTraj[:,i] - robot.jointState.jointAngles
    robot.plotError.append(jointPositonError)
    jointVelocityError = robot.TrajectoryPlan.jointVelTraj[:,i] - robot.jointState.jointVelocities
    
    torque = Kp.dot(jointPositonError) + Kd.dot(jointVelocityError) + robot.DynamicMatrices.gravityVector
    
    pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL, forces = torque)
    pb.stepSimulation()
    
plotError = np.array(robot.plotError)
print("i reached here")
robot.plotValues(plotError,time)
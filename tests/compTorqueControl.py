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
simTime = 30 # sec
trajTime = 10 #sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)

Kp = np.diag(6*[3])
Kd = np.diag(6*[2])

robot.turnOFFActuators()
robot.turnOFFInternalDamping()

robot.getJointsTrajectory(initjointAngles,finalJointAngles,trajTime,time)

print("final teaj is : ", robot.TrajectoryPlan.jointAngleTraj[:,-1])
for i in range(len(time)):
    
    robot.getJointInfo()
    robot.getForwardKinematics()
    robot.calculateJacobian()
    robot.getDyanamicMatrices()
    
    desiredAccel = robot.TrajectoryPlan.jointAccelTraj[:,i]
    positionError = Kp.dot(robot.TrajectoryPlan.jointAngleTraj[:,i] - robot.jointState.jointAngles)
    robot.plotError.append(robot.TrajectoryPlan.jointAngleTraj[:,i] - robot.jointState.jointAngles)
    velocityError = Kd.dot(robot.TrajectoryPlan.jointVelTraj[:,i] - robot.jointState.jointVelocities)
    
    inputAccel = desiredAccel + positionError + velocityError
    
    torque = robot.DynamicMatrices.massMatrix.dot(inputAccel) + robot.DynamicMatrices.coriolisVector + robot.DynamicMatrices.gravityVector
    
    pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL,forces = torque)
    pb.stepSimulation()
    
plotError = np.array(robot.plotError)
print("i reached here")
robot.plotValues(plotError,time)
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

desJointAngles = np.array([-2,-2.7,2.4,0,-1.57,-1.57])

desEndEffectorVel = np.array([0,0,0,0,0,0])

Kp = np.diag(6*[1])
Kd = np.diag(6*[1])

while True:
    robot.getJointInfo()
    robot.getForwardKinematics()
    robot.calculateJacobian()
    robot.getDyanamicMatrices()
    
    jointPositonError = desJointAngles - robot.jointState.jointAngles
    print(jointPositonError)
    jointVelocityError = desEndEffectorVel - robot.jointState.jointVelocities
    
    torque = Kp.dot(jointPositonError) + Kd.dot(jointVelocityError) + robot.DynamicMatrices.gravityVector
    
    pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL, forces = torque)
    pb.stepSimulation()
    
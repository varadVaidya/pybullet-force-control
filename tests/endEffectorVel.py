import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb


robot = Manipulator()

jointAngles = [0,-2,2,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)
desiredJointVelocityAngular = np.array([-0.0,0,-0.,0.,-0.,0.1])
desiredJointVelocityRPY = np.array([0,0,0.0,0.0,0.0,0.1])
while True:
    robot.getJointInfo()
    robot.getForwardKinematics()
    
    robot.calculateJacobian()
    #print("the geometric jacobian is:" , robot.Jacobian.geometricJacobian  )
    jointVelocityAngular = robot.Jacobian.geometricJacobianInv.dot(desiredJointVelocityAngular)
    #print(jointVelocity,"is the joint velocity")
    jointVelocityRPY = robot.Jacobian.analyticJacobianInv.dot(desiredJointVelocityRPY)
    
    #print(robot.Jacobian.analyticJacobian.dot(jointVelocityRPY))
    
    pb.setJointMotorControlArray(robot.armID , robot.controlJoints ,pb.VELOCITY_CONTROL,targetVelocities = jointVelocityRPY)
    pb.stepSimulation()
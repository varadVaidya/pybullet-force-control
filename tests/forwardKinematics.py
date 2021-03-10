import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np


robot = Manipulator()

robot.getForwardKinematics()
print(robot.forwardKinematics.linkPosition, "is the link position of the end effector" )
print(robot.forwardKinematics.linkOrientationQuaternion, "is the quaternion of the end effector" )
print(robot.forwardKinematics.linkOrientationRPY, "is the ROLL PTCH YAW of the end effector" )
print(robot.forwardKinematics.rotationMatrix, "is the rotation matirx of the end effector" )
jointAngles = [0,-1,1,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

robot.getForwardKinematics()

print(robot.forwardKinematics.linkPosition, "is the link position of the end effector" )
print(robot.forwardKinematics.linkOrientationQuaternion, "is the quaternion of the end effector" )
print(robot.forwardKinematics.linkOrientationRPY, "is the ROLL PTCH YAW of the end effector" )
print(robot.forwardKinematics.rotationMatrix, "is the rotation matirx of the end effector" )

'''

The values returned from the function are:

(0.7164617776870728, 0.109215646982193, 1.3644038438796997) is the link position of the end effector
(-0.49939653277397156, 0.5002074241638184, 0.500204861164093, 0.5001906156539917) is the quaternion of the end effector
(0.0, 1.5707963267948966, 1.5691739018207946) is the ROLL PTCH YAW of the end effector


Thus the roll pitch yaw follow the GLOBAL ROTATION ORDER OF X Y Z
that is:

roll around world x
pitch aroud world y
yaw around world Z

'''
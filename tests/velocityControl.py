import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb
#from ..manipulator.manipulator import Manipulator
import pybullet_utils.transformations as trans
#import tf.transformations as tF
## set the style for matplotli
robot = Manipulator()

jointAngles = [0.5,-1,2,-1.57,2,0]
robot.setJointAngles(jointAngles)

# sim time parameters
simTime = 30 # sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)


desEndEffector = np.array([0.3,0.2,1.5,2,1.3,3])
droll,dpitch,dyaw = desEndEffector[3:6]
desQuat = trans.unit_vector(trans.quaternion_from_euler(droll,dpitch,dyaw))

desEndEffectorVel = np.array([0,0,0,0,0,0])

# Kp = np.diag([.1,.1,.1,0.1,0.1,0.1])
# Kd = np.diag([0.1,0.1,0.1,0.1,0.1,0.1])

Kp = np.diag([1.6,1.6,1.6,0.8,0.8,0.8])
Kd = np.diag([0.48,0.48,0.48,0.24,0.24,0.24])


for i in range(len(time)):
    
    ##finding the state of the robot
    robot.getJointInfo()
    robot.getForwardKinematics()
    robot.calculateJacobian()
    
    endEffectorVel = robot.Jacobian.geometricJacobian.dot(robot.jointState.jointVelocities)
    
    positionError = desEndEffector[0:3] - np.array(robot.forwardKinematics.linkPosition)
    
    currQuat = trans.unit_vector(robot.forwardKinematics.linkOrientationQuaternion)
    
    errorQuat = trans.quaternion_multiply(desQuat,trans.quaternion_conjugate(currQuat))
    #print(errorQuat)
    orientationError = errorQuat[0:3] * np.sign(errorQuat[3])
    
    posError = np.hstack((positionError,orientationError))
    robot.plotError.append(posError)
    velError = desEndEffectorVel - endEffectorVel
    
    commandVelocity = Kp.dot(posError) + Kd.dot(velError)
    
    commandJointVelocity = robot.Jacobian.geometricJacobianInv.dot(commandVelocity)
    pb.setJointMotorControlArray(robot.armID , robot.controlJoints ,pb.VELOCITY_CONTROL,targetVelocities=commandJointVelocity)
    pb.stepSimulation()
    
plotError = np.array(robot.plotError)
print("i reached here")
robot.plotValues(plotError,time)
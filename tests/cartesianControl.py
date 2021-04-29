import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb
import pybullet_utils.transformations as trans
from time import sleep
#from ..manipulator.manipulator import Manipulator

## set the style for matplotli
robot = Manipulator()
pb.setRealTimeSimulation(False)
jointAngles = [0,-1.57,1.57,1,1.,1.]
robot.setJointAngles(jointAngles)

robot.turnOFFActuators()
robot.turnOFFInternalDamping()
robot.getJointInfo()
robot.getForwardKinematics()

desEndEffector = desEndEffector = np.array([0.3,0.2,1.5,2,0.2,1.2])
droll,dpitch,dyaw = desEndEffector[3:6]
desQuat = trans.unit_vector(trans.quaternion_from_euler(droll,dpitch,dyaw))

desEndEffectorVel = np.array([0,0,0,0,0,0])

Kp = np.diag([2.5,2.5,2.5,0.08,0.08,0.08])
Kd = np.diag([8,8,8,0.115,0.115,0.115])


# sim time parameters
simTime = 100 # sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)

f = open("plot.csv" , "w")
f.truncate()
f.close()
with open("plot.csv",'a') as f:
    
    for i in range(len(time)):
    
        robot.getJointInfo()
        robot.getForwardKinematics()
        robot.calculateJacobian()
        robot.getDyanamicMatrices()
        
        
        endEffectorVel = robot.Jacobian.geometricJacobian.dot(robot.jointState.jointVelocities)
        #print(endEffectorVel)
        positionError = desEndEffector[0:3] - np.array(robot.forwardKinematics.linkPosition)
        
        currQuat = trans.unit_vector(robot.forwardKinematics.linkOrientationQuaternion)
        
        errorQuat = trans.quaternion_multiply(desQuat,trans.quaternion_conjugate(currQuat))
        
        orientationError = errorQuat[0:3] * np.sign(errorQuat[3])
        
        posError = np.hstack((positionError,orientationError))

        np.savetxt(f, posError.reshape(1,6),newline='\n',fmt='%f',delimiter=',')
        #print("Pos Error",posError)
        robot.plotError.append(posError)
        
        
        
        #np.savetxt('plot.csv',robot.plotError,delimiter=',')
        velocityError = desEndEffectorVel - endEffectorVel
        
        commanadedForce = Kp.dot(posError) + Kd.dot(velocityError)
        
        commanadedJointTorque = robot.Jacobian.geometricJacobian.T.dot(commanadedForce) + robot.DynamicMatrices.gravityVector
        # robot.Jacobian.analyticJacobian.T.dot(commanadedForce)
        pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL, forces = commanadedJointTorque)
        pb.stepSimulation()
    
plotError = np.array(robot.plotError)
print("i reached here")
robot.plotValues(plotError,time)
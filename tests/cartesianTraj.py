import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb
import pybullet_utils.transformations as trans

#from ..manipulator.manipulator import Manipulator
## set the style for matplotli
robot = Manipulator()
table = pb.loadURDF("table/table.urdf", [0.,0.6,0], pb.getQuaternionFromEuler([0,0,0]))

pb.setRealTimeSimulation(False)
jointAngles = [0,-1.57,1.57,1,1.,1.]
robot.setJointAngles(jointAngles)

robot.turnOFFActuators()
robot.turnOFFInternalDamping()
robot.getJointInfo()
robot.getForwardKinematics()
desQuat = [1,0,-1,0]


point1 = robot.Waypoint(position=[0.3,0.2,1.5],velocity=[0,0,0],acceleration=[0,0,0])
point2 = robot.Waypoint(position=[0.3,0.6,0.6],velocity=[0,0,0],acceleration=[0,0,0])


# sim time parameters
simTime = 60 # sec
trajTime = 30 #sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)

positionTrajectory,velocityTrajectory,accelerationTrajectory = robot.getCartesianTraj(point1,point2,trajTime,time)

Kp = np.diag([2.5,2.5,2.5,0.08,0.08,0.08])
Kd = np.diag([8,8,8,0.115,0.115,0.115])
pb.enableJointForceTorqueSensor(robot.armID,robot.endEffectorIndex,True)

#print(positionTrajectory)

for i in range(len(time)):
    
    robot.getJointInfo()
    robot.getForwardKinematics()
    robot.calculateJacobian()
    robot.getDyanamicMatrices()
    
    endEffectorVel = robot.Jacobian.geometricJacobian.dot(robot.jointState.jointVelocities)
    
    ## hacky fix till i add orientation velocity:
    
    desVel = np.hstack((velocityTrajectory[i],[0,0,0]))
    
    positionError = positionTrajectory[i] - np.array(robot.forwardKinematics.linkPosition)
    
    currQuat = trans.unit_vector(robot.forwardKinematics.linkOrientationQuaternion)
        
    errorQuat = trans.quaternion_multiply(desQuat,trans.quaternion_conjugate(currQuat))
        
    orientationError = errorQuat[0:3] * np.sign(errorQuat[3])
    
    posError = np.hstack((positionError,orientationError))
    
    
    
    endEffectorforces = robot.jointForceTorqueSensorValues()
    #print(endEffectorforces)
    robot.plotError.append(endEffectorforces)
    
    #print(endEffectorforces[-1])
    
        
    #np.savetxt('plot.csv',robot.plotError,delimiter=',')
    velocityError = desVel - endEffectorVel
    
    commanadedForce = Kp.dot(posError) + Kd.dot(velocityError)
    
    commanadedJointTorque = robot.Jacobian.geometricJacobian.T.dot(commanadedForce) + robot.DynamicMatrices.gravityVector
    # robot.Jacobian.analyticJacobian.T.dot(commanadedForce)
    pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL, forces = commanadedJointTorque)
    pb.stepSimulation()
    
    
plotError = np.array(robot.plotError)
print("i reached here")
robot.plotValues(plotError,time)


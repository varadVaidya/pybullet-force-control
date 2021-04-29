import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb
import pybullet_utils.transformations as trans
#from ..manipulator.manipulator import Manipulator
#from ..manipulator.manipulator import Manipulator
## set the style for matplotlib
robot = Manipulator()
pb.setRealTimeSimulation(False)
jointAngles = [0,-2,2,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

robot.turnOFFActuators()
robot.turnOFFInternalDamping()
robot.getJointInfo()
robot.getForwardKinematics()
robot.calculateJacobian()
robot.getDyanamicMatrices()

desEndEffector = robot.forwardKinematics.linkState
# desEndEffector = np.array([0.3,-0.2,1.5,2,0.2,1.2])
# droll,dpitch,dyaw = desEndEffector[3:6]

droll,dpitch,dyaw = desEndEffector[3:6]
desQuat = trans.unit_vector(trans.quaternion_from_euler(droll,dpitch,dyaw))

desEndEffectorVel = np.array([0,0,0,0,0,0])

# sim time parameters
simTime = 400 # sec
timeSteps = simTime * 240
time = np.linspace(0,simTime,num=timeSteps)


K_MATRIX = np.diag([100,100,100,100,100,100])
D_MATRIX = 2 * np.sqrt(K_MATRIX)
#DES_INERTIA = robot.DynamicMatrices.cartesianMassMatrix
DES_INERTIA = 0.1 * np.eye(6)
initalForce = robot.controlZero
ForceGUIids = robot.ForceGUIcontrol(forces=initalForce, max_limit=10, min_limit=-10)


for i in range(len(time)):
    
    robot.getJointInfo()
    robot.getForwardKinematics()
    robot.calculateJacobian()
    robot.getDyanamicMatrices()
    
    
    F_ext = robot.readGUIparams(ForceGUIids) # applied external forces
    
    endEffectorVel = robot.Jacobian.geometricJacobian.dot(robot.jointState.jointVelocities)
    #print(endEffectorVel)
    positionError = desEndEffector[0:3] - np.array(robot.forwardKinematics.linkPosition)
    
    currQuat = trans.unit_vector(robot.forwardKinematics.linkOrientationQuaternion)
    
    errorQuat = trans.quaternion_multiply(desQuat,trans.quaternion_conjugate(currQuat))
    
    orientationError = errorQuat[0:3] * np.sign(errorQuat[3])
    
    posError = np.hstack((positionError,orientationError))
    
    #print("Pos Error",posError)
    #robot.plotError.append(posError)
    
    velocityError = desEndEffectorVel - endEffectorVel
    
    springForce = K_MATRIX.dot(posError) + D_MATRIX.dot(velocityError)
    
    taskSpaceForce = np.linalg.multi_dot((
                                         np.linalg.inv(DES_INERTIA), robot.DynamicMatrices.cartesianMassMatrix , springForce
    ))
    
    externalForce = ( np.linalg.inv(DES_INERTIA).dot(robot.DynamicMatrices.cartesianMassMatrix) - np.eye(6) ).dot(F_ext)
    
    commandedForce = taskSpaceForce + externalForce
    
    commanadedJointTorque = robot.Jacobian.geometricJacobian.T.dot(commandedForce) + robot.DynamicMatrices.gravityVector
    
    pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL, forces = commanadedJointTorque)
    pb.stepSimulation()
    



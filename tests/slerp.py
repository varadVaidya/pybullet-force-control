## aim of this test is to implement slerp method for quaternion

import numpy as np
import quaternion 
import sys,os
import pybullet as pb
from time import sleep
path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
#from ..manipulator.manipulator import Manipulator

q1 = np.quaternion(1,0,0,0)
# q2 = quaternion.from_euler_angles(np.array([1.2,1.2,1.2]))
q2 = np.quaternion(1,0,1,0)
trajtime = np.linspace(0,10,10)


slerp = quaternion.quaternion_time_series.slerp(q1, q2, 0, 10, trajtime)
test1 = quaternion.slerp_evaluate(q1,q2,0.1 )
slerp = quaternion.as_float_array(slerp)
slerpPybullet = np.empty_like(slerp)
slerpPybullet[:,3] = slerp[:,0]
slerpPybullet[:,0:3] = slerp[:,1:4] 

# robot = Manipulator()
# pb.setRealTimeSimulation(False)
# jointAngles = [0,-1.57,1.57,1,1.,1.]
# robot.setJointAngles(jointAngles)

# robot.turnOFFActuators()
# robot.turnOFFInternalDamping()
# robot.getJointInfo()
# robot.getForwardKinematics()


# currentPos = robot.forwardKinematics.linkPosition

# # stateMatrix = np.empty(100,7)

# # stateMatrix[:,0:3] = currentPos
# # stateMatrix[:,3:7] = slerpPybullet

# for i in range(len(slerpPybullet)):
#     robot.getJointInfo()
#     robot.getForwardKinematics()
#     robot.calculateJacobian()
#     robot.getDyanamicMatrices()
    
#     currentPos = robot.forwardKinematics.linkPosition
#     jointValues = pb.calculateInverseKinematics(robot.armID,robot.endEffectorIndex,currentPos,slerpPybullet[i])
    
#     pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.POSITION_CONTROL,targetPositions = jointValues)
#     sleep((0.1))
#     pb.stepSimulation()
    
    



#print(slerp)
print((slerpPybullet))
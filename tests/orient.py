import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)
    

import numpy as np
import pybullet as pb
from manipulator import Manipulator

###TODo: figure out the range of pybullet ROLL PITCH YAW


def RPYMatrix(rpy):
    
    cr, cp, cy = [np.cos(i) for i in rpy]
    sr, sp, sy = [np.sin(i) for i in rpy]
    R = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                  [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                  [-sp, cp*sr, cp*cr]])
    return R
    
def inverseRPY(R):
    
    r = np.arctan2(R[2, 1], R[2, 2])
    p = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    y = np.arctan2(R[1, 0], R[0, 0])

    
    return r,p,y

initalRPY =3.14 ,-2,-1.56

initalpybulletMatrix = np.array(pb.getMatrixFromQuaternion(pb.getQuaternionFromEuler(initalRPY))).reshape(3,3)
print("The inital RPY is:", initalRPY)
print("PybulletPRy Matrix: \n" , initalpybulletMatrix   )
#print("Calculated Matrix: \n" , RPYMatrix(initalRPY))
print("RPY from pybullet matrix: \n", inverseRPY(initalpybulletMatrix))
print("RPY fromour matrix: \n", inverseRPY(RPYMatrix(initalRPY)))
print("RPY from pybullet conversion: \n", pb.getEulerFromQuaternion(pb.getQuaternionFromEuler(initalRPY))) 
## the claculater PRY takes the shortest path and hence the error from subtraction will not be zero, even when the error is technically zero.

desRPY = -3 ,1,0
despybulletMatrix = np.array(pb.getMatrixFromQuaternion(pb.getQuaternionFromEuler(desRPY))).reshape(3,3)
print("The desired RPY is:", desRPY)
print("PybulletPRy Matrix: \n" , despybulletMatrix   )
print("RPY from pybullet conversion: \n", pb.getEulerFromQuaternion(pb.getQuaternionFromEuler(desRPY))) 
print("Calculated PRY from pybullet: \n ", inverseRPY(despybulletMatrix)) 


rotationMatrixError = despybulletMatrix.dot(initalpybulletMatrix.T)
RPYFromRotationError = inverseRPY(rotationMatrixError)
#print("the subtraction error is: ", subtractionError)
print("the rotation matrix error is: " , RPYFromRotationError)
print("Checking if the ERROR RPY is correct:: \n" , rotationMatrixError.dot(initalpybulletMatrix))
print("will it reach Rdes:" , np.array(pb.getMatrixFromQuaternion(pb.getQuaternionFromEuler(RPYFromRotationError))).reshape(3,3).dot(initalpybulletMatrix)  )






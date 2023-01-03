import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator
import numpy as np
import pybullet as pb
from time import sleep
## set the style for matplotli
robot = Manipulator()
table = pb.loadURDF("table/table.urdf", [0.35,0.35,0], pb.getQuaternionFromEuler([0,0,1.57]))
jointAngles = [1,-1.5,1.5,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

FREE_FALL = True

robot.turnOFFActuators()
## sets the arm in free fall as no torques is applied on the joints.
if FREE_FALL:
    torque = 6*[0]
    shift = [0, -0.02, 0]
    meshScale = [0.1, 0.1, 0.1]
    visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                    fileName="duck.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shift,
                                    meshScale=meshScale)
    
    collisionShapeId = pb.createCollisionShape(shapeType=pb.GEOM_MESH,
                                          fileName="duck_vhacd.obj",
                                          collisionFramePosition=shift,
                                          meshScale=meshScale)
    rangex = 10
    rangey = 10
    for i in range(rangex):
        for j in range(rangey):
            pb.createMultiBody(baseMass=1,
                            baseInertialFramePosition=[0, 0, 0],
                            baseCollisionShapeIndex=collisionShapeId,
                            baseVisualShapeIndex=visualShapeId,
                            basePosition=[((-rangex / 2) + i) * meshScale[0] * 2,
                                            (-rangey / 2 + j) * meshScale[1] * 2, 1],
                            useMaximalCoordinates=True)
    
    while True:
        
        pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL,forces = torque)
        sleep(0.01)
        pb.stepSimulation()
        
if not FREE_FALL:
    
    while True:
        
        rrobot.getJointInfo()
        robot.getForwardKinematics()
        robot.calculateJacobian()
        robot.getDyanamicMatrices()
        
        torque = robot.DynamicMatrices.gravityVector
        
        pb.setJointMotorControlArray(robot.armID,robot.controlJoints,pb.TORQUE_CONTROL,forces = torque)
        sleep(0.01)
        pb.stepSimulation()

#### the robot is able to hold itself against gravity.
### hence the ggravity vector calculation is correct.
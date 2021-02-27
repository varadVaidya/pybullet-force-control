####
## Pybullet simulation of UR5 robot
####

## import necessary libs
import numpy as np
import pybullet as pb
import pybullet_data
from time import sleep


class Manipulator():
    
    ## Container class to store joint state
    
    class JointStateInfo():
        def __init__(self, Angles = None ,Velocities = None ,ReactionForces = None):
            
            self.jointAngles = Angles
            self.jointVelocities = Velocities
            self.jointReactionForces = ReactionForces
            
            super().__init__()
        
        
    def __init__(self,basePosition = [0,0,0],baseOrientation = [0,0,0,1]):
        
        # start pybullet
        pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # set gravity 
        pb.setGravity(0,0,-9.81)
        pb.setRealTimeSimulation(False)
        #load URDF and plane
        self.armID = pb.loadURDF("urdf/ur5.urdf",basePosition,baseOrientation,useFixedBase = True)
        pb.loadURDF("plane.urdf")
        
        #define joints  
        self.controlJoints = []
        self.totalJoints = pb.getNumJoints(self.armID)
        print("Total Number of joints : ", self.totalJoints)
        baseName = pb.getBodyInfo(self.armID)
        print("Base Name is: ",baseName)
        
        # set the end effector link .
        self.endEffectorIndex = -1
        ## define the joints that we can control
        
        for i in range(self.totalJoints):
            jointInfo = pb.getJointInfo(self.armID,i)
            
            if jointInfo[2]==0:
                #append the joints we can control
                self.controlJoints.append(i)
                
            if jointInfo[1] == b'ee_fixed_joint':
                #set the endeffector joint
                self.endEffectorIndex = i 
        
        print(self.endEffectorIndex,"is the end effector index") 
        ## control zero is the zero vec required for variopus calculations.
        ## the length for the zero vector is the length of the control joints         
        self.controlZero = [0] * len(self.controlJoints)
        
        self.jointState = self.JointStateInfo()
        
        
    def setJointAngles(self,jointAngles):
        
        pb.setJointMotorControlArray(self.armID,self.controlJoints,pb.POSITION_CONTROL,targetPositions =jointAngles , targetVelocities = self.controlZero)
        
        for __ in range(200):
            pb.stepSimulation()
            sleep(0.05)
    
    def getJointInfo(self):
        '''
        sets the joint info in the joint Info class formed
        '''
        joint = pb.getJointStates(self.armID,self.controlJoints)
        
        jointAngles = [ i[0] for i in joint]
        jointVelocities = [ i[1] for i in joint]
        jointReactionForces = [ i[2] for i in joint]
        
        self.jointState.jointAngles = jointAngles
        self.jointState.jointVelocities = jointVelocities
        self.jointState.jointReactionForces = jointReactionForces
        
        
        
    
        
            
            
        
        
        
        

####
## Pybullet simulation of UR5 robot
####

## import necessary libs
import numpy as np
import pybullet as pb
import pybullet_data
from time import sleep
from math import sin,cos,pow
import matplotlib.pyplot as plt
from matplotlib import style
from matplotlib.animation import FuncAnimation


class Manipulator():
    
    
    ## Container class to store joint state
    class JointStateInfo():
        def __init__(self, Angles = None ,Velocities = None ,ReactionForces = None):
            
            self.jointAngles = Angles
            self.jointVelocities = Velocities
            self.jointReactionForces = ReactionForces
            
            
    
    ## Container class to store forward kinematics  
    class ForwardKinematics():
        
        def __init__(self, linkPosition = None, linkOrientationQuaternion = None, linkOrientationRPY = None ,rotationMatroix = None, linkStateRPY = None,
                     linkOrientationQuaternionXYZ = None , linkOrientationQuaternionW = None , linkOrientationQuaternionWXYZ = None):
            
            self.linkPosition = linkPosition
            self.linkOrientationQuaternion = linkOrientationQuaternion
            self.linkOrientationQuaternionXYZ = linkOrientationQuaternionXYZ
            self.linkOrientationQuaternionW = linkOrientationQuaternionW
            self.linkOrientationRPY = linkOrientationRPY
            self.linkOrientationQuaternionWXYZ = linkOrientationQuaternionWXYZ
            self.rotationMatrix = rotationMatroix
            
            self.linkState = linkStateRPY
            
            
    
    ## container class for Jacobain
    class RobotJacobian():
        
        def __init__(self , geometricJacobian = None , analyticJacobian = None , geometricJacobianInv = None , analyticJacobianInv = None ):
            
            self.geometricJacobian = geometricJacobian
            self.analyticJacobian  = analyticJacobian
            self.geometricJacobianInv = geometricJacobianInv
            self.analyticJacobianInv = analyticJacobianInv
            
        
    #container class to store dynamic matrices
    class RobotDynamicMatrices():
        
        def __init__(self, massMatrix = None, coriolisVector = None , gravityVector = None , cartesianMassMatrix = None) :
            
            self.massMatrix = massMatrix
            self.coriolisVector = coriolisVector
            self.gravityVector = gravityVector
            self.cartesianMassMatrix = cartesianMassMatrix
            
            
    
    ## container class to store desired hjoint angles:
    
    class desiredJointState():
        def __init__(self,jointAngleTraj = None,jointVelTraj = None,jointAccelTraj = None):
            
            self.jointAngleTraj = jointAngleTraj
            self.jointVelTraj = jointVelTraj
            self.jointAccelTraj = jointAccelTraj
    
    
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
        
        ## empty list to hold all the error values.
        self.plotError = []
        self.plotIndex = []     ##index list to storeas a time representative.
        
        self.XError , self.YError , self.ZError , self.RollError , self.PitchError , self.YawError = [],[],[],[],[],[]
        ## init the container class for joint state
        self.jointState = self.JointStateInfo()
        ## init the container class for forward kinematics
        self.forwardKinematics = self.ForwardKinematics()
        
        ## init the container class for the jacobian
        self.Jacobian = self.RobotJacobian()
        
        ## init the container class to store dynamic matrices.
        self.DynamicMatrices = self.RobotDynamicMatrices()
        
        ## init the container class to store trajectory
        self.TrajectoryPlan = self.desiredJointState()
        
        
    ## moves the robot to the desired joint angles for the simulatiuons
    def setJointAngles(self,jointAngles):
        
        pb.setJointMotorControlArray(self.armID,self.controlJoints,pb.POSITION_CONTROL,targetPositions =jointAngles , targetVelocities = self.controlZero)
        
        for __ in range(200):
            pb.stepSimulation()
            #sleep(0.005)
    
    ## calculates the jointInfo and sets it inside the container class
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
        
    ## calculate the forwardKinematics and store them inside the container class
    
    def getForwardKinematics(self):
        
        endEffector = pb.getLinkState(self.armID,self.endEffectorIndex,computeForwardKinematics=True)
        
        self.forwardKinematics.linkPosition = endEffector[4]
        self.forwardKinematics.linkOrientationQuaternion = endEffector[5]
        self.forwardKinematics.linkOrientationQuaternionXYZ = np.array(endEffector[5][0:3])
        self.forwardKinematics.linkOrientationQuaternionW = np.array(endEffector[5][3])
        #self.forwardKinematics.linkOrientationQuaternionWXYZ = np.array(endEffector[5][3] + endEffector[5][0:3])
        self.forwardKinematics.linkOrientationRPY = pb.getEulerFromQuaternion(endEffector[5])
        self.forwardKinematics.rotationMatrix = np.array(pb.getMatrixFromQuaternion(endEffector[5])).reshape(3,3)
        self.forwardKinematics.linkState = self.forwardKinematics.linkPosition + self.forwardKinematics.linkOrientationRPY
        
    def calculateJacobian(self):
        
        jointAngles = self.jointState.jointAngles
        
        endEffector = pb.getLinkState(self.armID,self.endEffectorIndex)
        
        linJac,angJac = pb.calculateJacobian(self.armID,self.endEffectorIndex,endEffector[2],jointAngles,self.controlZero,self.controlZero)
        geometricJacobian = np.vstack((linJac,angJac))
        
        self.Jacobian.geometricJacobian = geometricJacobian
        self.Jacobian.geometricJacobianInv = np.linalg.inv(geometricJacobian)
        
        ### now calculating analytical jacobain
        x,y,z = self.forwardKinematics.linkOrientationRPY
        
        ER_Matrix = np.array([
            [1,0,sin(y)],
            [0,cos(x), -cos(y)*sin(x)],
            [0,sin(x),cos(x)*cos(y)]
        ])
        
        ER_MatrixInv = np.linalg.inv(ER_Matrix)
        
        Ee_Matrix = np.block([
            [np.eye(3,3),     np.zeros((3,3))],
            [np.zeros((3,3)) , ER_MatrixInv   ]
        ])
        
        #Ee_MatirxInv = np.linalg.inv(Ee_Matrix)
        
        analyticJacobian = Ee_Matrix.dot(geometricJacobian)
        
        self.Jacobian.analyticJacobian = analyticJacobian
        self.Jacobian.analyticJacobianInv = np.linalg.inv(analyticJacobian)
    
    def getDyanamicMatrices(self):
        
        massMatrix = np.array(
            pb.calculateMassMatrix(self.armID,self.jointState.jointAngles)
        )
        
        gravityVector = np.array(
            pb.calculateInverseDynamics(self.armID,self.jointState.jointAngles,self.controlZero,self.controlZero)
        )
        
        coriolisGravityVector = np.array(
            pb.calculateInverseDynamics(self.armID,self.jointState.jointAngles,self.jointState.jointVelocities,self.controlZero)
        )
        coriolisVector = coriolisGravityVector - gravityVector
        
        ## calculate mass matrix in the cartesian space
        
        jacobianInv =  self.Jacobian.geometricJacobianInv
        
        cartesianMassMatrix = jacobianInv.T.dot(massMatrix.dot(jacobianInv))
                
        self.DynamicMatrices.massMatrix = massMatrix
        self.DynamicMatrices.gravityVector = gravityVector
        self.DynamicMatrices.coriolisVector = coriolisVector
        self.DynamicMatrices.cartesianMassMatrix = cartesianMassMatrix
    
    def turnOFFActuators(self):
        pb.setJointMotorControlArray(self.armID,self.controlJoints,pb.VELOCITY_CONTROL,forces = self.controlZero)
    
    def turnOFFInternalDamping(self):
        
        for i in self.controlJoints:
            pb.changeDynamics(self.armID,i, linearDamping = 0, angularDamping = 0 , jointDamping = 0 )
            
    def planJointTrajectory(self,initJointAngle,finalJointAngle,trajTime,simTimeArray):
        ## plan a cubic trajectory beetween two points in joint space.
        '''
        inputs to this function are:
        1. initalJointAngles:
        2. final joint angles
        3.  trajectory time
        
        '''
        jointAngleTraj = np.empty_like(simTimeArray)
        jointVelTraj = np.empty_like(simTimeArray)
        jointAccelTraj = np.empty_like(simTimeArray)
        
        simTime = trajTime # sec
        timeSteps = simTime * 240
        time = np.linspace(0,simTime,num=timeSteps)
        
        coeffMatrix = np.array([
            [0,0,0,1],
            [0,0,1,0],
            [pow(trajTime,3),pow(trajTime,2),trajTime,1],
            [3*pow(trajTime,2),2*trajTime,1,0]
        ])
        jointConstraint = np.array([initJointAngle,0,finalJointAngle,0])
        #                       th(0),   thdot(0), th(T) , thdot(T)
        jointCoeff = np.linalg.solve(coeffMatrix,jointConstraint)
        jointVelCoeff = [3*jointCoeff[0] , 2*jointCoeff[1], jointCoeff[2]]
        
        jointAngle = np.polyval(jointCoeff,time)
        jointVel = np.polyval(jointVelCoeff,time)
        jointAccel = np.zeros_like(time)
        
        jointAngleTraj[0:jointAngle.size] = jointAngle
        jointVelTraj[0:jointVel.size] = jointVel
        jointAccelTraj[0:jointAccel.size] = jointAccel
        
        jointAngleTraj[jointAngle.size:] = jointAngle[-1]
        jointVelTraj[jointVel.size:] = jointVel[-1]
        jointAccelTraj[jointAccel.size:] = jointAccel[-1]
        
        return jointAngleTraj,jointVelTraj,jointAccelTraj
    
    def getJointsTrajectory(self,initJointAngles,finalJointAngles,trajTime,simTimeArray):
        
        jointAngleTraj = []
        jointVelTraj = []
        jointAccelTraj = []
        
        for i in range(len(initJointAngles)):
            init,final = initJointAngles[i],finalJointAngles[i]
            AngleTraj,VelTraj,AccelTraj = self.planJointTrajectory(init,final,trajTime,simTimeArray)
            
            jointAngleTraj.append(AngleTraj)
            jointVelTraj.append(VelTraj)
            jointAccelTraj.append(AccelTraj)
        
        jointAngleTraj = np.array(jointAngleTraj)
        jointVelTraj = np.array(jointVelTraj)
        jointAccelTraj = np.array(jointAccelTraj)
        
        self.TrajectoryPlan.jointAngleTraj = jointAngleTraj
        self.TrajectoryPlan.jointVelTraj = jointVelTraj
        self.TrajectoryPlan.jointAccelTraj = jointAccelTraj
    
    def getMatrixFromRPY(self,rpy):
        cr, cp, cy = [np.cos(i) for i in rpy]
        sr, sp, sy = [np.sin(i) for i in rpy]
        R = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                  [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                  [-sp, cp*sr, cp*cr]])
        return R

    def getRPYFromMatrix(self,R):
        
        r = np.arctan2(R[2, 1], R[2, 2])
        p = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
        y = np.arctan2(R[1, 0], R[0, 0])
    
        return r,p,y
    
    def getRPYOrientationError(self,currentOrientationRPY, desiredOrientationRPY):

        desiredRotationMatrix = self.getMatrixFromRPY(desiredOrientationRPY)
        currentRotationMatrix = self.getMatrixFromRPY(currentOrientationRPY)
        
        errorRotationMatrix = desiredRotationMatrix.dot(currentRotationMatrix.T)
        
        errorRPY = self.getRPYFromMatrix(errorRotationMatrix) 
        
        return errorRPY
         
            
    def plotValues(self, plotError, time):
        
        style.use(['science' , 'std-colors'])
        
        fig = plt.figure()
        ax1 = fig.add_subplot(1,1,1)
        ax1.set_title('Error Over Time')
        
        
        ax1.plot(time,plotError[:,0] , label = 'X error' )
        ax1.plot(time,plotError[:,1] , label = 'Y error' )
        ax1.plot(time,plotError[:,2] , label = 'Z error' )
        ax1.plot(time,plotError[:,3] , label = 'Roll error' )
        ax1.plot(time,plotError[:,4] , label = 'Pitch error' )
        ax1.plot(time,plotError[:,5] , label = 'Yaw error' )
        plt.tight_layout()
        ax1.legend()
        
        plt.show()
    
    def writeToCSV(self,row):
        """
        this function will write to a csv file for continous plotting of the data in the row...
        
        THE row needs to be an 1D numpy array...
                
        """
    
    
    def readGUIparams(self, ids):
        val1 = pb.readUserDebugParameter(ids[0])
        val2 = pb.readUserDebugParameter(ids[1])
        val3 = pb.readUserDebugParameter(ids[2])
        val4 = pb.readUserDebugParameter(ids[3])
        val5 = pb.readUserDebugParameter(ids[4])
        val6 = pb.readUserDebugParameter(ids[5])
        return np.array([val1, val2, val3, val4, val5, val6])
    
    def ForceGUIcontrol(self, forces, max_limit = 10, min_limit = -10):
        fxId = pb.addUserDebugParameter("fx", min_limit, max_limit, forces[0]) #force along x
        fyId = pb.addUserDebugParameter("fy", min_limit, max_limit, forces[1]) #force along y
        fzId = pb.addUserDebugParameter("fz", min_limit, max_limit, forces[2]) #force along z
        mxId = pb.addUserDebugParameter("mx", min_limit, max_limit, forces[3]) #moment along x
        myId = pb.addUserDebugParameter("my", min_limit, max_limit, forces[4]) #moment along y
        mzId = pb.addUserDebugParameter("mz", min_limit, max_limit, forces[5]) #moment along z
        return [fxId, fyId, fzId, mxId, myId, mzId]
    # def animate(self,i):
    #     if self.plotError[-1] is not None:
    #         Xe,Ye,Ze,Re,Pe,Ye = self.plotError[-1]
    #     else:
    #         Xe,Ye,Ze,Re,Pe,Ye = 0,0,0,0,0,0
    #     self.XError.append(Xe)
    #     self.YError.append(Ye)
    #     self.ZError.append(Ze)
        
    #     self.RollError.append(Re)
    #     self.PitchError.append(Pe)
    #     self.YawError.append(Ye)
        
    #     plt.cla()
        
    #     plt.plot(self.plotIndex,self.XError,label = 'X error')
    #     plt.plot(self.plotIndex,self.YError,label = 'Y error')
    #     plt.plot(self.plotIndex,self.ZError,label = 'Z error')
    #     plt.plot(self.plotIndex,self.RollError,abel = 'Roll error')
    #     plt.plot(self.plotIndex,self.PitchError,label = 'Pitch error')
    #     plt.plot(self.plotIndex,self.YawError,label = 'Yaw error')
    
    # def livePlot(self):
    #     ani = FuncAnimation(plt.gcf(), self.animate , interval = 100)
    #     plt.show()
    
    
        
        
        
        
        
        
        
        
        

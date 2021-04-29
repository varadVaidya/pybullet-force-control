import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
style.use('science')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)


### this script will run in parallel to the main pybullet code..
### set the flag to give proper legends and title of the plot...

titleFlag = 0 
## set 0 for error...
## set 1 for end effector state
## set 2 for forces and torques..
print("I am doing something")

XErrors,YErrors,ZErrors,RollErrors,PitchErrors,YawErrors = [],[],[],[],[],[]

if titleFlag == 0:
    print("here")
    def animate(i):

        csv = np.loadtxt("plot.csv", dtype= float, delimiter= ',')
        
        #print(XErrors)
        # for row in csv:
        #     #print(len(row))
        #     if len(row) > 1 :
        #         x,y,z,roll,pitch,yaw = row
        #         XErrors.append(x)
        #         YErrors.append(y)
        #         ZErrors.append(z)
        #         RollErrors.append(roll)
        #         PitchErrors.append(pitch)
        #         YawErrors.append(yaw)

        
        ax1.clear()
        
        ax1.set_title('Error Over Time')
        ax1.plot(csv[:,0], label = 'X Error')  
        ax1.plot(csv[:,1], label = 'Y Error')
        ax1.plot(csv[:,2], label = 'Y Error')       
        ax1.plot(csv[:,3], label = 'Roll Error') 
        ax1.plot(csv[:,4],label = 'Pitch Error') 
        ax1.plot(csv[:,5],  label = 'Yaw Error') 
        ax1.legend()
        
    ani = animation.FuncAnimation(fig,animate,interval = 1000)
    plt.show()

if titleFlag == 1:
    
    def animate(i):

        csv = np.loadtxt("plot.csv", dtype= float, delimiter= ',')

        XPos,YPos,ZPos,RollPos,PitchPos,YawPos = [],[],[],[],[],[]
        for row in csv:

            if len(row) >1 :
                x,y,z,roll,pitch,yaw = row
                XPos.append(x)
                YPos.append(y)
                ZPos.append(z)
                RollPos.append(roll)
                PitchPos.append(pitch)
                YawPos.append(yaw)

        
        ax1.clear()
        
        ax1.set_title('Position State Over Time')
        ax1.plot(XPos, label = 'XPos')  
        ax1.plot(YPos, label = 'YPos')
        ax1.plot(ZPos, label = 'ZPos')       
        ax1.plot(RollPos, label = 'RollPos') 
        ax1.plot(PitchPos,label = 'PitchPos') 
        ax1.plot(YawPos,  label = 'YawPos') 
        ax1.legend()    
        
        ani = animation.FuncAnimation(fig,animate,interval = 100)
        plt.show()


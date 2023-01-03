import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
style.use('science')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def animate(i):
    
    csv = np.loadtxt("test.csv", dtype= float, delimiter= ',')
    
    xs = []
    ys = []
    count = 1
    for row in csv:
        
        if len(row) >1 :
            x = row[0]
            y = row[1]
            xs.append(x)
            ys.append(y)

    ax1.clear()
    ax1.plot(ys)  
    ax1.plot(xs)      
    
ani = animation.FuncAnimation(fig,animate,interval = 100)
plt.show()
            
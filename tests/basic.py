import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator

robot = Manipulator()

jointAngles = [0,-1,1,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

## checking the jointstate class
robot.getJointInfo()
print(robot.jointState.jointAngles)
## output was : [6.677375070206805e-07, -0.9999963432351262, 0.9999999992945597, -1.5699750628085123, -1.5700044660935193, -1.5699993390368723]
## which matches correctly with the provided joint values..

jointAngles = [0.5,-1,1,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

robot.getJointInfo()
print(robot.jointState.jointAngles)
## output was : [0.5000006673848731, -0.9999963439400562, 0.9999999999999996, -1.5699750639119205, -1.5700044672017706, -1.5699993401441568]
## which matches correctly with the provided joint values..

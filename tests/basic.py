import sys,os

path2add = os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, 'manipulator')))
if (not (path2add in sys.path)) :
    sys.path.append(path2add)

from manipulator import Manipulator

robot = Manipulator()

jointAngles = [0,-1,1,-1.57,-1.57,-1.57]
robot.setJointAngles(jointAngles)

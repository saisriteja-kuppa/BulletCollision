import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import IKConfig


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, -0.3])

# First let's define a class for the JointInfo.


robot = p.loadURDF(IKConfig.ROBOT, [0, 0, 0])
                    # flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT) 






# Let's analyze the R2D2 droid!
# print(f"robot unique ID: {robot}")

# print('-----------------------------------------------------')
# print(p.getNumJoints(robot))




# for i in range(p.getNumJoints(robot)):
#   joint = Joint(*p.getJointInfo(robot, i))
#   print(joint.index,joint.name)
#   print('------------------')



# print(getContactPoints, getClosestPoints)








while True:
    p.stepSimulation()
    pts = p.getContactPoints()
    print("num contacts = ", len(pts))
    for i in pts:
        for no,j in enumerate(list(i)):
            print
        print('points are ', i)
        print('---------------------------')
    time.sleep(1. / 240.)
    # break
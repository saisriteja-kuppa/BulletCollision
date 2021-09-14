import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import config
import numpy as np
from kinematics import ForwardKinematics, InverseKinematics
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})



import numpy

def getEquidistantPoints(p1, p2, parts):
    return zip(numpy.linspace(p1[0], p2[0], parts+1),
               numpy.linspace(p1[1], p2[1], parts+1))



#staubli
def movej(angs, robot, joint_indices = list(range(2,8,1))):
    p.setJointMotorControlArray(bodyIndex=robot,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=angs,
                                jointIndices=list(range(1,7,1))
                                )

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.loadURDF("plane.urdf", [0, 0, -0.3])



# robot_name =  'Elfin'
robot_name = 'Staubli'

pose = ForwardKinematics([0,0,-90,90,-90,0],robot_name)
ik_sols = InverseKinematics(pose,robot_name)


height = np.array([0.2] * len(ik_sols))
x = [0] * len(ik_sols)
y = np.linspace(-len(ik_sols)/2, len(ik_sols)/2, len(ik_sols))

positions = np.array([x,y ,height]).transpose()

robots_in_sim =[p.loadURDF(config.ROBOTS[robot_name], pos)  for pos in positions] 

for i  in range(len(ik_sols)):
    movej(ik_sols[i], robots_in_sim[i])
    print(np.rad2deg(ik_sols[i]))    


while (p.isConnected()):
  p.stepSimulation()
  


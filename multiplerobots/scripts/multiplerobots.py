import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import config
import numpy as np

def movej(angs, robot):
    p.setJointMotorControlArray(bodyIndex=robot,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=np.deg2rad(angs),
                                jointIndices=list(range(2,8,1))
                                )

p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, -0.3])


robot1 = p.loadURDF(config.ROBOTS['Elfin'], [1.5, 1.5, 0.2])
robot2 = p.loadURDF(config.ROBOTS['Staubli'], [1.5, -1.5, 0.2])
robot3 = p.loadURDF(config.ROBOTS['UR5e'], [-1.5, 1.5, 0.2])
robot4 = p.loadURDF(config.ROBOTS['KukaLBR'], [-1.5,-1.5, 0.2])


while (p.isConnected()):
  p.stepSimulation()


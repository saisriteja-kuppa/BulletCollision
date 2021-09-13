import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import IKConfig
import numpy as np

def movej(angs, robot):
    p.setJointMotorControlArray(bodyIndex=robot,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=np.deg2rad(angs),
                                jointIndices=list(range(2,8,1))
                                )

p.connect(p.GUI)


# clid = p.connect(p.SHARED_MEMORY)
# if (clid < 0):
#   p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, -0.3])

# First let's define a class for the JointInfo.


robot = p.loadURDF(IKConfig.ROBOT, [0, 0, 0.2])
                    # flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT) 




movej([0,0,-90,90,-90,0],robot)

# Let's analyze the R2D2 droid!
# print(f"robot unique ID: {robot}")

# print('-----------------------------------------------------')
# print(p.getNumJoints(robot))




# for i in range(p.getNumJoints(robot)):
#   joint = Joint(*p.getJointInfo(robot, i))
#   print(joint.index,joint.name)
#   print('------------------')



# print(getContactPoints, getClosestPoints)

useCollisionShapeQuery = False

geomBox = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])

geomBox2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])


baseOrientationB = p.getQuaternionFromEuler([0, 0.3, 0])  #[0,0.5,0.5,0]
basePositionB = [1.5, 0, 1]
obA = -1
obB = -1

obA = robot #p.createMultiBody(baseMass=0, baseCollisionShapeIndex=geom, basePosition=[0.5, 0, 1])
obB = p.createMultiBody(baseMass=0,
                        baseCollisionShapeIndex=geomBox,
                        basePosition=basePositionB,
                        baseOrientation=baseOrientationB)


baseOrientationB = p.getQuaternionFromEuler([0, 0, 0])
obB1 = p.createMultiBody(baseMass=0,
                        baseCollisionShapeIndex=geomBox2,
                        basePosition=[0.5,0,0.2],
                        baseOrientation=baseOrientationB)



lineWidth = 6
colorRGB = [1, 0, 0]
lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=colorRGB,
                            lineWidth=lineWidth,
                            lifeTime=0)



pitch = 0
yaw = 0

while (p.isConnected()):
  p.stepSimulation()
#   pitch += 0.01
#   if (pitch >= 3.1415 * 2.):
#     pitch = 0
#   yaw += 0.01
#   if (yaw >= 3.1415 * 2.):
#     yaw = 0
  pitch = 0.26
  yaw = 0.75

  baseOrientationB = p.getQuaternionFromEuler([yaw, pitch, 0])
  
  if (obB >= 0):
    p.resetBasePositionAndOrientation(obB, basePositionB, baseOrientationB)

  if (useCollisionShapeQuery):
    pts = p.getClosestPoints(bodyA=-1,
                             bodyB=-1,
                             distance=100,
                             collisionShapeA=robot,
                             collisionShapeB=geomBox,
                             collisionShapePositionA=[0.5, 0, 1],
                             collisionShapePositionB=basePositionB,
                             collisionShapeOrientationB=baseOrientationB)
    #pts = p.getClosestPoints(bodyA=obA, bodyB=-1, distance=100, collisionShapeB=geomBox, collisionShapePositionB=basePositionB, collisionShapeOrientationB=baseOrientationB)
  else:
    pts = p.getClosestPoints(bodyA=obA, bodyB=obA, linkIndexA = 7,linkIndexB = 2, distance=100)
    pts1 = p.getClosestPoints(bodyA=obA, bodyB=obB, linkIndexA = 7, distance=100)


  if len(pts) > 0:
    distance = pts[0][8]
    ptA = pts[0][5]
    ptB = pts[0][6]
    p.addUserDebugLine(lineFromXYZ=ptA,
                       lineToXYZ=ptB,
                       lineColorRGB=colorRGB,
                       lineWidth=lineWidth,
                       lifeTime=0,
                       replaceItemUniqueId=lineId)
    
  pts = pts1
  if len(pts) > 0:
    distance = pts[0][8]
    ptA = pts[0][5]
    ptB = pts[0][6]
    p.addUserDebugLine(lineFromXYZ=ptA,
                       lineToXYZ=ptB,
                       lineColorRGB=colorRGB,
                       lineWidth=lineWidth,
                       lifeTime=0,
                       replaceItemUniqueId=lineId)
    
    
    
    
  pts = p.getContactPoints()
  print("num contacts = ", len(pts))
  for i in pts:
        for no,j in enumerate(list(i)):
            print
        print('points are ', i)
        print('---------------------------')
  #time.sleep(1./240.)

#removeCollisionShape is optional:
#only use removeCollisionShape if the collision shape is not used to create a body
#and if you want to keep on creating new collision shapes for different queries (not recommended)
# p.removeCollisionShape(geom)
p.removeCollisionShape(geomBox)





# while True:
#     p.stepSimulation()
#     pts = p.getClosestPoints(bodyA=robot, bodyB=robot,linkIndexA = 3,linkIndexB= 8, distance=1000)
#     print(pts)
#     # pts = p.getContactPoints()
#     # print("num contacts = ", len(pts))
#     # for i in pts:
#     #     for no,j in enumerate(list(i)):
#     #         print
#     #     print('points are ', i)
#     #     print('---------------------------')
#     time.sleep(1. / 240.)
#     # break




     
     


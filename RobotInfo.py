import pybullet as p
from dataclasses import dataclass
import IKConfig



@dataclass
class Joint:
  index: int
  name: str
  type: int
  gIndex: int
  uIndex: int
  flags: int
  damping: float
  friction: float
  lowerLimit: float
  upperLimit: float
  maxForce: float
  maxVelocity: float
  linkName: str
  axis: tuple
  parentFramePosition: tuple
  parentFrameOrientation: tuple
  parentIndex: int

  def __post_init__(self):
    self.name = str(self.name, 'utf-8')
    self.linkName = str(self.linkName, 'utf-8')



def GetJointsInformation(robot):
    joints = dict()
    for i in range(p.getNumJoints(robot)):
        joint = Joint(*p.getJointInfo(robot, i))
        # print(joint.index,joint.name)
        joints[joint.index] = joint.name
    return joints


# def GetLinksInformation(robot):
#     links = dict()
#     for i in range(p.getNumLinks(robot)):
#         link = p.getLinkInfo(robot, i)
#         links[link.index] = link.name
#     return links

        

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.DIRECT)
        

robot = p.loadURDF(IKConfig.ROBOT, [0, 0, 0])        
        
# #Joints information
joints_info =   GetJointsInformation(robot)
print(joints_info)


# Links information
# links_info  = GetLinksInformation(robot)
# print(links_info)
        

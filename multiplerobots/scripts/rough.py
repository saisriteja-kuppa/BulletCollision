import numpy as np
from kinematics import ForwardKinematics, InverseKinematics


import numpy

def getEquidistantPoints(p1, p2, parts):
    return zip(numpy.linspace(p1[0], p2[0], parts+1),
               numpy.linspace(p1[1], p2[1], parts+1))


pose = ForwardKinematics(np.deg2rad([0,0,-90,90,-90,0]))   
ik_sols = InverseKinematics(pose)
print(pose, ik_sols)


height = np.array([0.2] * len(ik_sols))
xy_pos = np.array(list(getEquidistantPoints((1,1), (len(ik_sols),len(ik_sols)), len(ik_sols)-1)))




positions = np.array([xy_pos[:,0] ,xy_pos[:,0] ,height])

print(positions.transpose())
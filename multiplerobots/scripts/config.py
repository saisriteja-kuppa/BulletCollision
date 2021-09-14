import os

PathToUrdf = '/home/saisriteja/BulletPhysics/Pose/multiplerobots/urdfFiles'


ROBOTS = dict()


#path to URDF
ROBOTS['Elfin'] = os.path.join(PathToUrdf, 'elfin5/urdf/elfin5.urdf')
ROBOTS['Staubli'] = os.path.join(PathToUrdf,'staublirx160/urdf/rx160.urdf')  
ROBOTS['UR5e'] = os.path.join(PathToUrdf,'ur5e/urdf/ur5e.urdf')
ROBOTS['KukaLBR'] = os.path.join(PathToUrdf,'kukalbr/urdf/lbr_iiwa_14_r820.urdf')




RobotIKfast = '/home/saisriteja/BulletPhysics/Pose/multiplerobots/ikfast'




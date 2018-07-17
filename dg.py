import pinocchio as se3
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
import time
import matplotlib.pylab as plt
plt.ion()

f=open('/home/datchuth/Dev/matlab/extractFromBags/bagfiles/HRP2/testPattern/test_pattern_stepping0_astate.log')
#f=open('/home/datchuth/Dev/matlab/extractFromBags/bagfiles/HRP2/testPattern/test_pattern_stepping1_astate.log')
qs = []
for l in f.readlines():
    try:
        x= map(float,filter(lambda x: len(x)>0,l.split(' ')))
        if len(x) == 176: qs.append(x)
    except:pass
qs = np.matrix(qs)[:,:30]


urdf = '/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14.urdf'
pkgs = [ '/opt/openrobots/share',]

#(self, filename, package_dirs=package_dirs, root_joint=se3.JointModelFreeFlyer())

robot = RobotWrapper(urdf,package_dirs=pkgs,root_joint=se3.JointModelFreeFlyer())
#robot.initDisplay()#loadModel=True)

T = qs.shape[0]
RLEG = qs[:,:6]
LLEG = qs[:,6:12]
CHEST = qs[:,12:14]
HEAD  = qs[:,14:16]
RARM  = qs[:,16:23]
LARM  = qs[:,23:]
FF = qs[:,:7]*0
FF[:,6] = 1

qs = np.hstack( [ FF,CHEST,HEAD,LARM,RARM,LLEG,RLEG ])

ll = 25
rl = 31
d = zero([T,3])

#compute first oMi for right foot

q = qs[0,:].T
se3.forwardKinematics(robot.model,robot.data,q)
oMr0 = robot.data.oMi[rl].copy()

#print q.T,oMr0
#print '='*50
for i,q in enumerate(qs):
    #FF = qs[i,:7]
    q = q.T
    #robot.display(q); time.sleep(1e-3)
    se3.forwardKinematics(robot.model,robot.data,q)
    #oMl = robot.data.oMi[ll]
    oMr = robot.data.oMi[rl]
    r0Mr = oMr0.inverse()*oMr
    d[i,:] = r0Mr.translation.T
    #print d[i,:],q.T,oMr0,oMr
    #if i>10: break    
    
np.savetxt('/home/datchuth/Dev/matlab/extractFromBags/bagfiles/HRP2/testPattern/test_pattern_stepping0_rfoot2.txt',d)

# -*- coding: utf-8 -*-
"""
Created on Sat Jul 21 16:14:07 2018

@author: datchuth
"""


import pinocchio as se3
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
import time
import matplotlib.pylab as plt
from IPython import embed
plt.ion()


def rMl(robot):
    oMr = robot.data.oMi[rl].copy()
    oMl = robot.data.oMi[ll].copy()
    return oMr.inverse().copy() * oMl
    
def lMr(robot):
    return rMl(robot).inverse()
   
f=open('/home/datchuth/Dev/matlab/extractFromBags/bagfiles/HRP2/July2018/testPattern/testPattern0_2018_07_18_astateDemo.log')
#f=open('/home/datchuth/Dev/matlab/extractFromBags/bagfiles/HRP2/testPattern/test_pattern_stepping1_astate.log')
#f=open('/home/datchuth/Dev/matlab/extractFromBags/bagfiles/HRP2/July2018/hwalk/hwalk0_2018_07_18_astateDemo.log')
qs = []
for l in f.readlines():
    try:
        x= map(float,filter(lambda x: len(x)>0,l.split(' ')))
        if len(x) == 176: qs.append(x)
    except:pass
Force = np.matrix(qs)[:,120:132]
ForceRZ = Force[:,2]
ForceLZ = Force[:,8]
time = np.matrix(qs)[:,174]
qs = np.matrix(qs)[:,:30]


def hyst(x, th_lo, th_hi, initial = False):
    hi = x >= th_hi
    lo_or_hi = (x <= th_lo) | hi
    ind = np.nonzero(lo_or_hi)[0]
    if not ind.size: # prevent index error if ind is empty
        return np.zeros_like(x, dtype=bool) | initial
    cnt = np.cumsum(lo_or_hi) # from 0 to len(x)
    return np.where(cnt, hi[ind[cnt-1]], initial)

RorL = hyst(ForceRZ.A1-ForceLZ.A1,-200,200)

#plt.plot(ForceRZ)
#plt.plot(ForceLZ)
#plt.plot(RorL*500)
#plt.show()


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
dl = zero([T,3])
dr = zero([T,3])
debug = zero([T,3])
#compute first oMi for right foot

q = qs[0,:].T
se3.forwardKinematics(robot.model,robot.data,q)
oMr0 = robot.data.oMi[rl].copy()

#print q.T,oMr0
#print '='*50

oMl = robot.data.oMi[ll].copy()
oMr = robot.data.oMi[rl].copy()

for i,q in enumerate(qs):
    #FF = qs[i,:7]
    q = q.T
    #robot.display(q); time.sleep(1e-3)
    se3.forwardKinematics(robot.model,robot.data,q)
    bMl = robot.data.oMi[ll].copy()
    bMr = robot.data.oMi[rl].copy()
    
    if(RorL[i]):
        #pied droit appui
        oMr = oMr
        oMl = oMr * rMl(robot)
    else:
        oMl = oMl
        oMr = oMl * lMr(robot)
    #log the feet position
    dr[i,:] = (oMr0.inverse() * oMr).translation.T
    dl[i,:] = oMl.translation.T
    debug[i,:] = rMl(robot).translation.T
    #pied gauche appui

plt.figure()    
#plt.plot(dl)
plt.plot(dr)
plt.figure()
plt.plot(debug)
plt.plot(RorL)
plt.show()

#embed()
np.savetxt('/home/datchuth/Dev/matlab/extractFromBags/bagfiles/HRP2/July2018/testPattern/testPattern0_rfoot.txt',dr)
#np.savetxt('/home/datchuth/Dev/matlab/extractFromBags/bagfiles/HRP2/July2018/hwalk/hwalk0_rfoot.txt',dr)

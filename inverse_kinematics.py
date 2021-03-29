import sympy as sp
from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax',pretty_print=False)
from sympy.physics.mechanics import dynamicsymbols
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import drawRobotics as dR
from matplotlib.widgets import Slider
import time
import datetime
axcolor = 'lightgoldenrodyellow'
th1Init = 0
l2Init = 300
th3Init = 0

org0 = np.array([[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])
L1 = 500
L2 = 300
def calcORGs(theta1,l2,theta3):
    theta1 = dR.conv2Rad(theta1)
    theta3 = dR.conv2Rad(theta3)

    T01 = np.dot(dR.D_q(0, 0, 2 * L1), dR.RotZ(theta1))
    T12 = dR.D_q(L1, (L1 - l2), 0)
    T23 = np.dot(dR.D_q(L2, 0, 0), dR.RotX(-theta3))


    org1 = np.dot(org0,T01)
    org2 = np.dot(org1,T12)
    org3 = np.dot(org2,T23)


    return org1,org2,org3

def drawObject(org1,org2,org3):
    dR.drawPointWithAxis(ax, org0, lineStyle='--', vectorLength=1, lineWidth=2)
    dR.drawPointWithAxis(ax, org1, vectorLength=1)
    dR.drawPointWithAxis(ax, org2, vectorLength=1)
    dR.drawPointWithAxis(ax, org3, vectorLength=1)

    dR.drawVector(ax, org0, org1, arrowstyle='-', lineColor='c', proj=False, lineWidth=10)
    dR.drawVector(ax, org1, org2, arrowstyle='-', lineColor='k', proj=False, lineWidth=8)
    dR.drawVector(ax, org2, org3, arrowstyle='-', lineColor='k', proj=False, lineWidth=6)

    ax.set_xlim([-1000,1000]), ax.set_ylim([-1000,1000]), ax.set_zlim([0,2000])
    ax.set_xlabel('X axis'), ax.set_ylabel('Y axis'), ax.set_zlabel('Z axis')

def update(num,th1Angle,L2Length,lines):
    theta1 = th1Angle
    l2 = L2Length
    theta3 = 0
    i = num

    org1,org2,org3 = calcORGs(theta1[i],l2[i],theta3)
    ax.cla()
    lines = drawObject(org1,org2,org3)
    return lines

org1, org2, org3 = calcORGs(th1Init,l2Init,th3Init)
fig = plt.figure(figsize=(13,9))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.3)

Px = 850
Py = 100

lines = drawObject(org1,org2,org3)
th1Angle = [np.rad2deg(math.atan2(Px,Py)-math.atan2(L1+L2,np.sqrt(Px**2+Py**2-(L1+L2)**2))),np.rad2deg(math.atan2(Px,Py)-math.atan2(L1+L2,-np.sqrt(Px**2+Py**2-(L1+L2)**2)))]
L2Length = [L1+np.sqrt(-(L1+L2)**2+Px**2+Py**2),L1-np.sqrt(-(L1+L2)**2+Px**2+Py**2)]

line_ani = animation.FuncAnimation(fig, update, len(th1Angle), fargs = (th1Angle,L2Length,lines),interval=1000,repeat=True,cache_frame_data=False)


ax.view_init(azim=-150,elev=30)
# plt.savefig('fwd.png')
plt.show()


# left_px = T05[0, 3]  # Position of x
# left_py = T05[1, 3]  # Position of y
# left_pz = T05[2, 3]  # Position of z


#
#

# theta1 = sp.rad(0)
# l2 = L1
# theta3 = -sp.rad(70)
# theta4 = sp.rad(np.linspace(-60,60,60))
# l5 = 100
# px = []
# py = []
# pz = []
# for i in range(len(theta4)):                                                    # px,py,pz coords
#     px.append(leftarm_px(theta1, L1, L2, L4, l2, l5, theta3, theta4[i]))
#     py.append(leftarm_py(theta1, L1, L2, L4, l2, l5, theta3, theta4[i]))
#     pz.append(leftarm_pz(theta1, L1, L2, L4, l2, l5, theta3, theta4[i]))


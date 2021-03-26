from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax',pretty_print=False)
import numpy as np
import matplotlib.pyplot as plt
import drawRobotics as dR
from matplotlib.widgets import Slider
axcolor = 'lightgoldenrodyellow'
th1Init = 0
l2Init = 300
th3Init = 0
th4Init = 0
l5Init = 100
d2Init = 300
ap3Init = 0
ap4Init = 0
d5Init = 100
org0 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
L1 = 500
L2 = 10
L4 = 400
def calcORGs(theta1,l2,theta3,theta4,l5,d2,alpha3,alpha4,d5):
    theta1 = dR.conv2Rad(theta1)
    theta3 = dR.conv2Rad(theta3)
    theta4 = dR.conv2Rad(theta4)
    alpha3 = dR.conv2Rad(alpha3)
    alpha4 = dR.conv2Rad(alpha4)
    T01 = np.dot(dR.D_q(0, 0, 2 * L1), dR.RotZ(theta1))
    T12 = dR.D_q(L1, (L1 - l2), 0)
    T23 = np.dot(dR.D_q(L2, 0, 0), dR.RotX(-theta3))
    T34 = dR.RotZ(theta4)
    T45 = np.dot(dR.D_q(0, -L4, 0), dR.D_q(0, -l5, 0))
    M12 = dR.D_q(-L1, (L1 - d2), 0)
    M23 = np.dot(dR.D_q(-L2, 0, 0), dR.RotX(alpha3))
    M34 = dR.RotZ(alpha4)
    M45 = np.dot(dR.D_q(0, -L4, 0), dR.D_q(0, -d5, 0))

    org1 = np.dot(org0,T01)
    org2 = np.dot(org1,T12)
    org3 = np.dot(org2,T23)
    org4 = np.dot(org3,T34)
    org5 = np.dot(org4,T45)
    Rorg2 = np.dot(org1,M12)
    Rorg3 = np.dot(Rorg2,M23)
    Rorg4 = np.dot(Rorg3,M34)
    Rorg5 = np.dot(Rorg4,M45)
    return org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5

def drawObject(org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5):
    dR.drawPointWithAxis(ax, org0, lineStyle='--', vectorLength=1, lineWidth=2)
    dR.drawPointWithAxis(ax, org1, vectorLength=1)
    dR.drawPointWithAxis(ax, org2, vectorLength=1)
    dR.drawPointWithAxis(ax, org3, vectorLength=1)
    dR.drawPointWithAxis(ax, org4, vectorLength=1)
    dR.drawPointWithAxis(ax, org5, vectorLength=1, lineWidth=2)
    dR.drawPointWithAxis(ax, Rorg2, vectorLength=1)
    dR.drawPointWithAxis(ax, Rorg3, vectorLength=1)
    dR.drawPointWithAxis(ax, Rorg4, vectorLength=1)
    dR.drawPointWithAxis(ax, Rorg5, vectorLength=1, lineWidth=2)

    dR.drawVector(ax, org0, org1, arrowstyle='-', lineColor='c', proj=False, lineWidth=10)
    dR.drawVector(ax, org1, org2, arrowstyle='-', lineColor='k', proj=False, lineWidth=8)
    dR.drawVector(ax, org2, org3, arrowstyle='-', lineColor='k', proj=False, lineWidth=6)
    dR.drawVector(ax, org3, org4, arrowstyle='-', lineColor='k', proj=False, lineWidth=4)
    dR.drawVector(ax, org4, org5, arrowstyle='-', lineColor='k', proj=False, lineWidth=2)
    dR.drawVector(ax, org1, Rorg2, arrowstyle='-', lineColor='k', proj=False, lineWidth=8)
    dR.drawVector(ax, Rorg2, Rorg3, arrowstyle='-', lineColor='k', proj=False, lineWidth=6)
    dR.drawVector(ax, Rorg3, Rorg4, arrowstyle='-', lineColor='k', proj=False, lineWidth=4)
    dR.drawVector(ax, Rorg4, Rorg5, arrowstyle='-', lineColor='k', proj=False, lineWidth=2)

    ax.set_xlim([-1000,1000]), ax.set_ylim([-1000,1000]), ax.set_zlim([0,2000])
    ax.set_xlabel('X axis'), ax.set_ylabel('Y axis'), ax.set_zlabel('Z axis')

def update(val):
    theta1 = th1Angle.val
    l2 = L2Length.val
    theta3 = L3Angle.val
    theta4 = L4Angle.val
    l5 = L5Length.val
    d2 = R2Length.val
    alpha3 = R3Angle.val
    alpha4 = R4Angle.val
    d5 = R5Length.val

    org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5 = calcORGs(theta1,l2,theta3,theta4,l5,d2,alpha3,alpha4,d5)
    ax.cla()
    drawObject(org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5)

org1, org2, org3, org4, org5, Rorg2, Rorg3, Rorg4, Rorg5 = calcORGs(th1Init,l2Init,th3Init,th4Init,l5Init,d2Init,ap3Init,ap4Init,d5Init)
fig = plt.figure(figsize=(13,9))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.3)

th1Angle = plt.axes([0.125, 0.25, 0.77, 0.02])
l2Length = plt.axes([0.125, 0.22, 0.77, 0.02])
th3Angle = plt.axes([0.125, 0.19, 0.77, 0.02])
th4Angle = plt.axes([0.125, 0.16, 0.77, 0.02])
l5Length = plt.axes([0.125, 0.13, 0.77, 0.02])
d2Length = plt.axes([0.125, 0.10, 0.77, 0.02])
ap3Angle = plt.axes([0.125, 0.07, 0.77, 0.02])
ap4Angle = plt.axes([0.125, 0.04, 0.77, 0.02])
d5Length = plt.axes([0.125, 0.01, 0.77, 0.02])

th1Angle = Slider(th1Angle, r'$ \theta_1 $', -180.0, 180.0, valinit=th1Init)
L2Length = Slider(l2Length, r'$ l_L2 $', 200, 800, valinit=l2Init)
L3Angle = Slider(th3Angle, r'$ \theta_L3 $', -180.0, 180.0, valinit=th3Init)
L4Angle = Slider(th4Angle, r'$ \theta_L4 $', -180.0, 180.0, valinit=th4Init)
L5Length = Slider(l5Length, r'$ \l_L5 $', 0, 400, valinit=l5Init)
R2Length = Slider(d2Length, r'$ l_R2 $', 200, 800, valinit=d2Init)
R3Angle = Slider(ap3Angle, r'$ \theta_R3 $', -180.0, 180.0, valinit=ap3Init)
R4Angle = Slider(ap4Angle, r'$ \theta_R4 $', -180.0, 180.0, valinit=ap4Init)
R5Length = Slider(d5Length, r'$ \l_R5 $', 0, 400, valinit=d5Init)


drawObject(org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5)



th1Angle.on_changed(update)
L2Length.on_changed(update)
L3Angle.on_changed(update)
L4Angle.on_changed(update)
L5Length.on_changed(update)
R2Length.on_changed(update)
R3Angle.on_changed(update)
R4Angle.on_changed(update)
R5Length.on_changed(update)

ax.view_init(azim=-150,elev=30)
plt.show()



from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax',pretty_print=False)
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import drawRobotics as dR

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
    M23 = np.dot(dR.D_q(-L2, 0, 0), dR.RotX(-alpha3))
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
th1 = 0
L2Length = 300
L3Angle = 0
L4Angle = 0
L5Length = 100
R2Length = 300
R3Angle = 0
R4Angle = 0
R5Length = 100
def update(num,th1,L2Length,L3Angle,L4Angle,L5Length,R2Length,R3Angle,R4Angle,R5Length,lines):
    theta1 = th1
    l2 = L2Length
    theta3 = L3Angle
    theta4 = L4Angle
    l5 = L5Length
    d2 = R2Length
    alpha3 = R3Angle
    alpha4 = R4Angle
    d5 = R5Length
    i=num
    org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5 = calcORGs(theta1[i],l2[i],theta3[i],theta4[i],l5[i],d2[i],alpha3[i],alpha4[i],d5[i])
    ax.cla()
    lines = drawObject(org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5)

    return lines


org1, org2, org3, org4, org5, Rorg2, Rorg3, Rorg4, Rorg5 = calcORGs(th1Init,l2Init,th3Init,th4Init,l5Init,d2Init,ap3Init,ap4Init,d5Init)
fig = plt.figure(figsize=(13,9))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.3)


lines = drawObject(org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5)
th1 = np.concatenate((      np.linspace(0,0,30),    np.linspace(0,0,30),    np.linspace(0,0,30),    np.linspace(0,90,30),   np.linspace(90,90,30)))
L2Length = np.concatenate(( np.linspace(300,300,30),np.linspace(300,700,30),np.linspace(700,700,30),np.linspace(700,700,30),np.linspace(700,700,30)))
L3Angle = np.concatenate((  np.linspace(0,-45,30),  np.linspace(-45,-45,30),np.linspace(-45,0,30),  np.linspace(0,0,30),    np.linspace(0,-45,30)))
L4Angle = np.concatenate((  np.linspace(0,0,30),    np.linspace(0,0,30),    np.linspace(0,0,30),    np.linspace(0,0,30),    np.linspace(0,0,30)))
L5Length = np.concatenate(( np.linspace(100,100,30),np.linspace(100,100,30),np.linspace(100,100,30),np.linspace(100,100,30),np.linspace(100,100,30)))
R2Length = np.concatenate(( np.linspace(300,300,30),np.linspace(300,700,30),np.linspace(700,700,30),np.linspace(700,700,30),np.linspace(700,700,30)))
R3Angle = np.concatenate((  np.linspace(0,-45,30),  np.linspace(-45,-45,30),np.linspace(-45,0,30),  np.linspace(0,0,30),    np.linspace(0,-45,30)))
R4Angle = np.concatenate((  np.linspace(0,0,30),    np.linspace(0,0,30),    np.linspace(0,0,30),    np.linspace(0,0,30),    np.linspace(0,0,30)))
R5Length = np.concatenate(( np.linspace(100,100,30),np.linspace(100,100,30),np.linspace(100,100,30),np.linspace(100,100,30),np.linspace(100,100,30)))

line_ani = animation.FuncAnimation(fig, update, len(th1), fargs = (th1,L2Length,L3Angle,L4Angle,L5Length,R2Length,R3Angle,R4Angle,R5Length,lines),interval=100,repeat=False,cache_frame_data=False)

ax.view_init(azim=-150,elev=30)
plt.show()

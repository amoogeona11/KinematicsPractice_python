import caljaco as dR
import numpy as np
import sympy as sp
import numpy.linalg as lin
import math
import warnings
from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax',pretty_print=False)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from sympy.physics.mechanics import dynamicsymbols
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)
def LeftJacobian(th1,l_2,th3,th4,l_5):
    theta1, L1, L2, L4, l2, l5, theta3, theta4, theta, alpha, a, d = sp.symbols('theta1, L1, L2, L4, l2, l5, theta3, theta4, theta, alpha, a, d')
    rot = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0],
                     [sp.sin(theta) * sp.cos(alpha), sp.cos(theta) * sp.cos(alpha), -sp.sin(alpha)],
                     [sp.sin(theta) * sp.sin(alpha), sp.cos(theta) * sp.sin(alpha), sp.cos(alpha)]])  # Rotation Matrix
    trans = sp.Matrix([a, -d * sp.sin(alpha), d * sp.cos(alpha)])  # Porg Matrix
    last_row = sp.Matrix([[0, 0, 0, 1]])  # Bottom
    m = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)  # Translation Matrix

    org_0 = m.subs({alpha: 0, a: 0, theta: theta1, d: 2 * L1})  # 0->1 Translation Matrix
    org_1 = m.subs({alpha: sp.rad(90), a: L1, theta: sp.rad(90), d: l2 - L1})  # 1->2 Translation Matrix
    org_2 = m.subs({alpha: sp.rad(90), a: 0, theta: sp.rad(90) + theta3, d: L2})  # 2->3 Translation Matrix
    org_3 = m.subs({alpha: sp.rad(90), a: 0, theta: sp.rad(90) + theta4, d: 0})  # 3->4 Translation Matrix
    org_4 = m.subs({alpha: sp.rad(90), a: 0, theta: -sp.rad(90), d: l5 + L4})  # 4->5 Translation Matrix
    org1 = org_0
    org2 = np.dot(org1,org_1)
    org3 = np.dot(org2,org_2)
    org4 = np.dot(org3,org_3)
    org5 = np.dot(org4,org_4)

    X_L = org5[0,3]
    Y_L = org5[1,3]
    Z_L = org5[2,3]

    Jv = np.array([[sp.diff(X_L,l2),sp.diff(X_L,theta3),sp.diff(X_L,theta4),sp.diff(X_L,l5)],
                    [sp.diff(Y_L,l2),sp.diff(Y_L,theta3),sp.diff(Y_L,theta4),sp.diff(Y_L,l5)],
                    [sp.diff(Z_L,l2),sp.diff(Z_L,theta3),sp.diff(Z_L,theta4),sp.diff(Z_L,l5)]])
    Jw = np.array(np.hstack([np.vstack([0,0,0]), np.vstack(org_3[0:3,2]), np.vstack(org_4[0:3,2]), np.vstack([0,0,0])]))
    JMat = np.vstack([Jv, Jw])

    JMat = sp.lambdify((theta1, L1, L2, L4, l2, theta3, theta4, l5), JMat, 'math')
    X_L = sp.lambdify((theta1, L1, L2, L4, l2, theta3, theta4, l5), X_L, 'math')
    Y_L = sp.lambdify((theta1, L1, L2, L4, l2, theta3, theta4, l5), Y_L, 'math')
    Z_L = sp.lambdify((theta1, L1, L2, L4, l2, theta3, theta4, l5), Z_L, 'math')

    theta1 = th1
    l2 = l_2
    theta3 = th3
    theta4 = th4
    l5 = l_5
    L1 = 500
    L2 = 10
    L4 = 400
    J_Mat = np.array(JMat(theta1, L1, L2, L4, l2, theta3, theta4, l5), dtype=float)
    X_L = X_L(theta1, L1, L2, L4, l2, theta3, theta4, l5)
    Y_L = Y_L(theta1, L1, L2, L4, l2, theta3, theta4, l5)
    Z_L = Z_L(theta1, L1, L2, L4, l2, theta3, theta4, l5)
    return float(X_L), float(Y_L), float(Z_L), J_Mat

def RightJacobian(th1,l_2,th3,th4,l_5):
    theta1, L1, L2, L4, l2, l5, theta3, theta4, theta, alpha, a, d = sp.symbols('theta1, L1, L2, L4, l2, l5, theta3, theta4, theta, alpha, a, d')
    rot = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0],
                     [sp.sin(theta) * sp.cos(alpha), sp.cos(theta) * sp.cos(alpha), -sp.sin(alpha)],
                     [sp.sin(theta) * sp.sin(alpha), sp.cos(theta) * sp.sin(alpha), sp.cos(alpha)]])  # Rotation Matrix
    trans = sp.Matrix([a, -d * sp.sin(alpha), d * sp.cos(alpha)])  # Porg Matrix
    last_row = sp.Matrix([[0, 0, 0, 1]])  # Bottom
    m = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)  # Translation Matrix

    org_0 = m.subs({alpha: 0, a: 0, theta: theta1, d: 2 * L1})  # 0->1 Translation Matrix
    org_1 = m.subs({alpha: sp.rad(90), a: -L1, theta: sp.rad(90), d: l2 - L1})  # 1->2 Translation Matrix
    org_2 = m.subs({alpha: sp.rad(90), a: 0, theta: sp.rad(90) + theta3, d: L2})  # 2->3 Translation Matrix
    org_3 = m.subs({alpha: sp.rad(90), a: 0, theta: sp.rad(90) + theta4, d: 0})  # 3->4 Translation Matrix
    org_4 = m.subs({alpha: sp.rad(90), a: 0, theta: -sp.rad(90), d: l5 + L4})  # 4->5 Translation Matrix
    org1 = org_0
    org2 = np.dot(org1,org_1)
    org3 = np.dot(org2,org_2)
    org4 = np.dot(org3,org_3)
    org5 = np.dot(org4,org_4)

    X_L = org5[0,3]
    Y_L = org5[1,3]
    Z_L = org5[2,3]

    Jv = np.array([[sp.diff(X_L,l2),sp.diff(X_L,theta3),sp.diff(X_L,theta4),sp.diff(X_L,l5)],
                    [sp.diff(Y_L,l2),sp.diff(Y_L,theta3),sp.diff(Y_L,theta4),sp.diff(Y_L,l5)],
                    [sp.diff(Z_L,l2),sp.diff(Z_L,theta3),sp.diff(Z_L,theta4),sp.diff(Z_L,l5)]])
    Jw = np.array(np.hstack([np.vstack([0,0,0]), np.vstack(org_3[0:3,2]), np.vstack(org_4[0:3,2]), np.vstack([0,0,0])]))
    JMat = np.vstack([Jv, Jw])

    JMat = sp.lambdify((theta1, L1, L2, L4, l2, theta3, theta4, l5), JMat, 'math')
    X_L = sp.lambdify((theta1, L1, L2, L4, l2, theta3, theta4, l5), X_L, 'math')
    Y_L = sp.lambdify((theta1, L1, L2, L4, l2, theta3, theta4, l5), Y_L, 'math')
    Z_L = sp.lambdify((theta1, L1, L2, L4, l2, theta3, theta4, l5), Z_L, 'math')

    theta1 = th1
    l2 = l_2
    theta3 = th3
    theta4 = th4
    l5 = l_5
    L1 = 500
    L2 = 10
    L4 = 400
    J_Mat = np.array(JMat(theta1, L1, L2, L4, l2, theta3, theta4, l5), dtype=float)
    X_L = X_L(theta1, L1, L2, L4, l2, theta3, theta4, l5)
    Y_L = Y_L(theta1, L1, L2, L4, l2, theta3, theta4, l5)
    Z_L = Z_L(theta1, L1, L2, L4, l2, theta3, theta4, l5)
    return float(X_L), float(Y_L), float(Z_L), J_Mat
def LeftcalcORGs(th1,l_2,th3,th4,l_5):
    theta_1, L1, L2, L4, len2, len5, theta_3, theta_4, thetap, alphap, ap, dp = sp.symbols('theta_1, L1, L2, L4, len2, len5, theta_3, theta_4, thetap, alphap, ap, dp')
    rot = sp.Matrix([[sp.cos(thetap), -sp.sin(thetap), 0],
                     [sp.sin(thetap) * sp.cos(alphap), sp.cos(thetap) * sp.cos(alphap), -sp.sin(alphap)],
                     [sp.sin(thetap) * sp.sin(alphap), sp.cos(thetap) * sp.sin(alphap), sp.cos(alphap)]])  # Rotation Matrix
    trans = sp.Matrix([ap, -dp * sp.sin(alphap), dp * sp.cos(alphap)])  # Porg Matrix
    last_row = sp.Matrix([[0, 0, 0, 1]])  # Bottom
    n = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)  # Translation Matrix

    org0_temp = n.subs({alphap: 0, ap: 0, thetap: theta_1, dp: 2 * L1})  # 0->1 Translation Matrix
    org1_temp = n.subs({alphap: sp.rad(90), ap: L1, thetap: sp.rad(90), dp: len2 - L1})  # 1->2 Translation Matrix
    org2_temp = n.subs({alphap: sp.rad(90), ap: 0, thetap: sp.rad(90) + theta_3, dp: L2})  # 2->3 Translation Matrix
    org3_temp = n.subs({alphap: sp.rad(90), ap: 0, thetap: sp.rad(90) + theta_4, dp: 0})  # 3->4 Translation Matrix
    org4_temp = n.subs({alphap: sp.rad(90), ap: 0, thetap: -sp.rad(90), dp: len5 + L4})  # 4->5 Translation Matrix
    # T0 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T1 = np.array(org0_temp)
    T2 = np.array(np.dot(T1, org1_temp))
    T3 = np.array(np.dot(T2, org2_temp))
    T4 = np.array(np.dot(T3, org3_temp))
    T5 = np.array(np.dot(T4, org4_temp))

    T1 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T1, 'math')
    T2 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T2, 'math')
    T3 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T3, 'math')
    T4 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T4, 'math')
    T5 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T5, 'math')
    theta_1 = th1
    len2 = l_2
    theta_3 = th3
    theta_4 = th4
    len5 = l_5
    L1 = 500
    L2 = 10
    L4 = 400
    org1 = np.array(T1(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    org2 = np.array(T2(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    org3 = np.array(T3(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    org4 = np.array(T4(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    org5 = np.array(T5(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    return org1, org2, org3, org4, org5

def RightcalcORGs(th1,l_2,th3,th4,l_5):
    theta_1, L1, L2, L4, len2, len5, theta_3, theta_4, thetap, alphap, ap, dp = sp.symbols('theta_1, L1, L2, L4, len2, len5, theta_3, theta_4, thetap, alphap, ap, dp')
    rot = sp.Matrix([[sp.cos(thetap), -sp.sin(thetap), 0],
                     [sp.sin(thetap) * sp.cos(alphap), sp.cos(thetap) * sp.cos(alphap), -sp.sin(alphap)],
                     [sp.sin(thetap) * sp.sin(alphap), sp.cos(thetap) * sp.sin(alphap), sp.cos(alphap)]])  # Rotation Matrix
    trans = sp.Matrix([ap, -dp * sp.sin(alphap), dp * sp.cos(alphap)])  # Porg Matrix
    last_row = sp.Matrix([[0, 0, 0, 1]])  # Bottom
    n = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)  # Translation Matrix

    org0_temp = n.subs({alphap: 0, ap: 0, thetap: theta_1, dp: 2 * L1})  # 0->1 Translation Matrix
    org1_temp = n.subs({alphap: sp.rad(90), ap: -L1, thetap: sp.rad(90), dp: len2 - L1})  # 1->2 Translation Matrix
    org2_temp = n.subs({alphap: sp.rad(90), ap: 0, thetap: sp.rad(90) + theta_3, dp: L2})  # 2->3 Translation Matrix
    org3_temp = n.subs({alphap: sp.rad(90), ap: 0, thetap: sp.rad(90) + theta_4, dp: 0})  # 3->4 Translation Matrix
    org4_temp = n.subs({alphap: sp.rad(90), ap: 0, thetap: -sp.rad(90), dp: len5 + L4})  # 4->5 Translation Matrix
    # T0 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T1 = np.array(org0_temp)
    T2 = np.array(np.dot(T1, org1_temp))
    T3 = np.array(np.dot(T2, org2_temp))
    T4 = np.array(np.dot(T3, org3_temp))
    T5 = np.array(np.dot(T4, org4_temp))

    T1 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T1, 'math')
    T2 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T2, 'math')
    T3 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T3, 'math')
    T4 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T4, 'math')
    T5 = sp.lambdify((theta_1, L1, L2, L4, len2, theta_3, theta_4, len5), T5, 'math')
    theta_1 = th1
    len2 = l_2
    theta_3 = th3
    theta_4 = th4
    len5 = l_5
    L1 = 500
    L2 = 10
    L4 = 400
    org1 = np.array(T1(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    org2 = np.array(T2(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    org3 = np.array(T3(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    org4 = np.array(T4(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    org5 = np.array(T5(theta_1, L1, L2, L4, len2, theta_3, theta_4, len5))
    return org1, org2, org3, org4, org5
def Left_get_ik_sol(Leftgoal,Left_origin_state,iter):
    Lx_orig, Ly_orig, Lz_orig, garb0 = LeftJacobian(0,Left_origin_state[0],Left_origin_state[1],Left_origin_state[2],Left_origin_state[3])
    Lpos0 = np.array([Lx_orig, Ly_orig, Lz_orig, 0, 0, 0])
    Lx = np.linspace(Lx_orig, Leftgoal[0], iter)
    Ly = np.linspace(Ly_orig, Leftgoal[1], iter)
    Lz = np.linspace(Lz_orig, Leftgoal[2], iter)
    Lx_traj = [Lx_orig]
    Ly_traj = [Ly_orig]
    Lz_traj = [Lz_orig]
    theta1_tr = np.array([0])
    Ll2_tr = np.array([Left_origin_state[0]])
    Ltheta3_tr = np.array([Left_origin_state[1]])
    Ltheta4_tr = np.array([Left_origin_state[2]])
    Ll5_tr = np.array([Left_origin_state[3]])
    Left_prev_state = np.vstack(Left_origin_state)
    coeff = 1.01
    prior_error = 0
    for i in range(len(Lx)):
        Lx_temp, Ly_temp, Lz_temp, Ljaco0 = LeftJacobian(0,Left_prev_state[0],Left_prev_state[1],Left_prev_state[2],Left_prev_state[3])
        Left_goal_pos = np.array([Lx[i], Ly[i], Lz[i],0,0,0])
        Lx_traj.append(Lx_temp)
        Ly_traj.append(Ly_temp)
        Lz_traj.append(Lz_temp)
        id = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        Ldls_temp = Ljaco0.transpose()@Ljaco0+(coeff**2)*id
        Ldls = lin.inv(Ldls_temp)@Ljaco0.transpose()
        Lvec_e = Left_goal_pos-Lpos0
        left_e = np.vstack(Lvec_e)
        error = lin.norm(left_e)
        Left_delta_theta = Ldls@left_e
        theta1_tr = np.append(theta1_tr,0)
        Ll2_tr = np.append(Ll2_tr, Left_prev_state[0]+Left_delta_theta[0])
        Ltheta3_tr = np.append(Ltheta3_tr, Left_prev_state[1]+Left_delta_theta[1])
        Ltheta4_tr = np.append(Ltheta4_tr, Left_prev_state[2]+Left_delta_theta[2])
        Ll5_tr = np.append(Ll5_tr,Left_prev_state[3]+Left_delta_theta[3])
        Left_prev_state = [Left_prev_state[j] + Left_delta_theta[j][0] for j in range(len(Left_delta_theta))]
        if abs(error) > abs(prior_error):
            coeff += 0.3
        else:
            coeff -= 0.3

        prior_error = error

        Lpos0 = [Lx_temp,Ly_temp,Lz_temp,0,0,0]
    return theta1_tr, Ll2_tr, Ltheta3_tr, Ltheta4_tr, Ll5_tr

def Right_get_ik_sol(Rightgoal,Right_origin_state,iter):
    Rx_orig, Ry_orig, Rz_orig, garb0 = RightJacobian(0,Right_origin_state[0],Right_origin_state[1],Right_origin_state[2],Right_origin_state[3])
    Rpos0 = np.array([Rx_orig, Ry_orig, Rz_orig, 0, 0, 0])
    Rx = np.linspace(Rx_orig, Rightgoal[0], iter)
    Ry = np.linspace(Ry_orig, Rightgoal[1], iter)
    Rz = np.linspace(Rz_orig, Rightgoal[2], iter)
    Rx_traj = [Rx_orig]
    Ry_traj = [Ry_orig]
    Rz_traj = [Rz_orig]
    theta1_tr = np.array([0])
    Rl2_tr = np.array([Right_origin_state[0]])
    Rtheta3_tr = np.array([Right_origin_state[1]])
    Rtheta4_tr = np.array([Right_origin_state[2]])
    Rl5_tr = np.array([Right_origin_state[3]])
    Right_prev_state = np.vstack(Right_origin_state)
    prior_error = 0
    coeff = 1.01

    for i in range(len(Rx)):
        Rx_temp, Ry_temp, Rz_temp, Rjaco0 = RightJacobian(0,Right_prev_state[0],Right_prev_state[1],Right_prev_state[2],Right_prev_state[3])
        Right_goal_pos = np.array([Rx[i], Ry[i], Rz[i],0,0,0])
        Rx_traj.append(Rx_temp)
        Ry_traj.append(Ry_temp)
        Rz_traj.append(Rz_temp)
        id = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        Rdls_temp = Rjaco0.transpose()@Rjaco0+(coeff**2)*id
        Rdls = lin.inv(Rdls_temp)@Rjaco0.transpose()
        Rvec_e = Right_goal_pos-Rpos0
        right_e = np.vstack(Rvec_e)
        error = lin.norm(right_e)

        Right_delta_theta = Rdls@right_e
        theta1_tr = np.append(theta1_tr,0)
        Rl2_tr = np.append(Rl2_tr, Right_prev_state[0] + Right_delta_theta[0])
        Rtheta3_tr = np.append(Rtheta3_tr, Right_prev_state[1] + Right_delta_theta[1])
        Rtheta4_tr = np.append(Rtheta4_tr, Right_prev_state[2] + Right_delta_theta[2])
        Rl5_tr = np.append(Rl5_tr, Right_prev_state[3] + Right_delta_theta[3])
        Right_prev_state = [Right_prev_state[j] + Right_delta_theta[j][0] for j in range(len(Right_delta_theta))]
        if abs(error) > abs(prior_error):
            coeff += 0.3
        else:
            coeff -= 0.3

        prior_error = error
        Rpos0 = [Rx_temp,Ry_temp,Rz_temp,0,0,0]
    return theta1_tr, Rl2_tr, Rtheta3_tr, Rtheta4_tr, Rl5_tr


def drawObject(org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5):
    org0 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
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

def update(num,th1,L2Length,L3Angle,L4Angle,L5Length,R2Length,R3Angle,R4Angle,R5Length,lines):
    theta1 = th1
    l2 = L2Length
    theta3 = L3Angle
    theta4 = L4Angle
    l5 = L5Length
    R2 = R2Length
    Rtheta3 = R3Angle
    Rtheta4 = R4Angle
    R5 = R5Length
    i=num
    org1,org2,org3,org4,org5 = LeftcalcORGs(theta1[i],l2[i],theta3[i],theta4[i],l5[i])
    org1,Rorg2,Rorg3,Rorg4,Rorg5 = RightcalcORGs(theta1[i],R2[i],Rtheta3[i],Rtheta4[i],R5[i])
    ax.cla()
    lines = drawObject(org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5)
    return lines

print("Enter Destination : ex) 600 600 1200")
L_xgoal, L_ygoal, L_zgoal = map(float,input().split())
R_xgoal, R_ygoal, R_zgoal = map(float,input().split())
Ldestination = [L_xgoal,L_ygoal,L_zgoal]
Rdestination = [R_xgoal,R_ygoal,R_zgoal]
Left_origin_state = [float(300),float(0),float(0),float(100)]
Right_origin_state = [float(300),float(0),float(0),float(100)]
Left_xorg, Left_yorg, Left_zorg, dump = LeftJacobian(0,Left_origin_state[0],Left_origin_state[1],Left_origin_state[2],Left_origin_state[3])
Right_xorg, Right_yorg, Right_zorg, dump = RightJacobian(0,Right_origin_state[0],Right_origin_state[1],Right_origin_state[2],Right_origin_state[3])
Ldeparture = np.array([Left_xorg,Left_yorg,Left_zorg])
Rdeparture = np.array([Right_xorg,Right_yorg,Right_zorg])
print(Ldeparture,Rdeparture)
print(Ldestination,Rdestination)
trace_th1, Ltrace_l2, Ltrace_th3, Ltrace_th4, Ltrace_l5 = Left_get_ik_sol(Ldestination,Left_origin_state,50)
trace_th1, Rtrace_l2, Rtrace_th3, Rtrace_th4, Rtrace_l5 = Right_get_ik_sol(Rdestination,Right_origin_state,50)
print(trace_th1)
print(Ltrace_l2)
print(Ltrace_th3)
print(Ltrace_th4)
print(Ltrace_l5)
print(Rtrace_l2)
print(Rtrace_th3)
print(Rtrace_th4)
print(Rtrace_l5)

th1Init = float(0)
Ll2Init = float(300)
Lth3Init = float(0)
Lth4Init = float(0)
Ll5Init = float(100)
Rl2Init = float(300)
Rth3Init = float(0)
Rth4Init = float(0)
Rl5Init = float(100)
org1, org2, org3, org4, org5 = LeftcalcORGs(th1Init,Ll2Init,Lth3Init,Lth4Init,Ll5Init)
org1, Rorg2, Rorg3, Rorg4, Rorg5 = RightcalcORGs(th1Init,Rl2Init,Rth3Init,Rth4Init,Rl5Init)

fig = plt.figure(figsize=(13, 9))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.3)
lines = drawObject(org1,org2,org3,org4,org5,Rorg2,Rorg3,Rorg4,Rorg5)
line_ani = animation.FuncAnimation(fig, update, len(trace_th1), fargs = (trace_th1,Ltrace_l2,Ltrace_th3,Ltrace_th4,Ltrace_l5,Rtrace_l2,Rtrace_th3,Rtrace_th4,Rtrace_l5,lines),interval=100,repeat=False,cache_frame_data=False)
ax.view_init(azim=-150,elev=30)
plt.legend()
plt.show()

#
# x,y,z,Jaco = Jacobian(0,300,0,0,100)
# print(x,y,z)
# pin = np.array(lin.pinv(Jaco))
# pin = np.round(pin,5)
# print(Jaco)
# print(pin)





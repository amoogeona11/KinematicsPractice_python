import sympy as sp
from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax',pretty_print=False)
from sympy.physics.mechanics import dynamicsymbols
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
theta1, L1, L2, L4, l2, l5, theta3, theta4, theta, alpha, a, d, th1p,l2p,th3p,th4p,l5p = sp.symbols('theta1, L1, L2, L4, l2, l5, theta3, theta4, theta, alpha, a, d, theta1_prime, l2_prime, theta3_prime, theta4_prime, l5_prime')
rot = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0],
                 [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), -sp.sin(alpha)],
                 [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha), sp.cos(alpha)]])                        # Rotation Matrix
trans = sp.Matrix([a, -d*sp.sin(alpha), d*sp.cos(alpha)])                    # Porg Matrix
last_row = sp.Matrix([[0, 0, 0, 1]])                                        # Bottom
m = sp.Matrix.vstack(sp.Matrix.hstack(rot,trans),last_row)                  # Translation Matrix

m01 = m.subs({alpha:0, a:0, theta:theta1, d:2*L1})                          # 0->1 Translation Matrix
m12 = m.subs({alpha:sp.rad(90), a:L1, theta:sp.rad(90), d:l2-L1})           # 1->2 Translation Matrix
m23 = m.subs({alpha:sp.rad(90), a:0, theta:sp.rad(90)+theta3, d:L2})                   # 2->3 Translation Matrix
m34 = m.subs({alpha:sp.rad(90), a:0, theta:sp.rad(90)+theta4, d:0})                    # 3->4 Translation Matrix
m45 = m.subs({alpha:sp.rad(90), a:0, theta:-sp.rad(90), d:l5+L4})           # 4->5 Translation Matrix
m02 = m01*m12
m03 = m02*m23
m04 = m03*m34
m05 = m04*m45                                                   # 0->5 Translation Matrix
# T = sp.Matrix([[sp.trigsimp([m05[0,0]]),sp.trigsimp([m05[0,1]]),sp.trigsimp([m05[0,2]])],
#               [sp.trigsimp([m05[1,0]]),sp.trigsimp([m05[1,1]]),sp.trigsimp([m05[1,2]])],
#               [sp.trigsimp([m05[2,0]]),sp.trigsimp([m05[2,1]]),sp.trigsimp([m05[2,2]])]])
jv0 = sp.Matrix([sp.diff(m05[0,3],theta1),sp.diff(m05[0,3],l2),sp.diff(m05[0,3],theta3),sp.diff(m05[0,3],theta4),sp.diff(m05[0,3],l5)]).transpose()
jv1 = sp.Matrix([sp.diff(m05[1,3],theta1),sp.diff(m05[1,3],l2),sp.diff(m05[1,3],theta3),sp.diff(m05[1,3],theta4),sp.diff(m05[1,3],l5)]).transpose()
jv2 = sp.Matrix([sp.diff(m05[2,3],theta1),sp.diff(m05[2,3],l2),sp.diff(m05[2,3],theta3),sp.diff(m05[2,3],theta4),sp.diff(m05[2,3],l5)]).transpose()
jw = sp.Matrix.hstack(m01[0:3,2],m02[0:3,2],m03[0:3,2],m04[0:3,2],m05[0:3,2])
jaco = sp.Matrix.vstack(jv0,jv1,jv2,jw)
var = sp.Matrix([th1p,l2p,th3p,th4p,l5p])
velocity_Matrix = jaco*var
sp.pprint(jaco)

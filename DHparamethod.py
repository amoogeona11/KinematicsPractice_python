import sympy as sp
from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax',pretty_print=False)
from sympy.physics.mechanics import dynamicsymbols
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
theta1, L1, L2, L4, l2, l5, theta3, theta4, theta, alpha, a, d = sp.symbols('theta1, L1, L2, L4, l2, l5, theta3, theta4, theta, alpha, a, d')
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
m05 = m01*m12*m23*m34*m45                                                   # 0->5 Translation Matrix
# T = sp.Matrix([[sp.trigsimp([m05[0,0]]),sp.trigsimp([m05[0,1]]),sp.trigsimp([m05[0,2]])],
#               [sp.trigsimp([m05[1,0]]),sp.trigsimp([m05[1,1]]),sp.trigsimp([m05[1,2]])],
#               [sp.trigsimp([m05[2,0]]),sp.trigsimp([m05[2,1]]),sp.trigsimp([m05[2,2]])]])

T = sp.Matrix([[m05[0,0].simplify(),m05[0,1].simplify(),m05[0,2].simplify(),m05[0,3].simplify()],
              [m05[1,0].simplify(),m05[1,1].simplify(),m05[1,2].simplify(),m05[1,3].simplify()],
              [m05[2,0].simplify(),m05[2,1].simplify(),m05[2,2].simplify(),m05[2,3].simplify()],
              [m05[3,0].simplify(),m05[3,1].simplify(),m05[3,2].simplify(),m05[3,3].simplify()]
              ])
left_px = T[0,3]                                                                 # Position of x
left_py = T[1,3]                                                                 # Position of y
left_pz = T[2,3]                                                                 # Position of z

leftarm_px=sp.lambdify((theta1, L1, L2, L4, l2, l5, theta3, theta4),left_px,'math')     # Make formula
leftarm_py=sp.lambdify((theta1, L1, L2, L4, l2, l5, theta3, theta4),left_py,'math')
leftarm_pz=sp.lambdify((theta1, L1, L2, L4, l2, l5, theta3, theta4),left_pz,'math')


L1 = 500
L2 = 10
L4 = 300
theta1 = sp.rad(0)
l2 = L1
theta3 = -sp.rad(70)
theta4 = sp.rad(np.linspace(-60,60,60))                                         # Desired motion
l5 = 100
px = []
py = []
pz = []
for i in range(len(theta4)):                                                    # px,py,pz coords
    px.append(leftarm_px(theta1, L1, L2, L4, l2, l5, theta3, theta4[i]))
    py.append(leftarm_py(theta1, L1, L2, L4, l2, l5, theta3, theta4[i]))
    pz.append(leftarm_pz(theta1, L1, L2, L4, l2, l5, theta3, theta4[i]))
px = np.array(px)
py = np.array(py)
pz = np.array(pz)

fig = plt.figure()
ax = Axes3D(fig)
line = plt.plot(px,py,pz)
ax.set_xlim(-1000,1000); ax.set_ylim(-1000,1000); ax.set_zlim(0,1500);
plt.savefig('wave.png')
plt.show()


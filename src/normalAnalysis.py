import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.msg import ContactsState
rospy.init_node("Test", anonymous=True)


# get state
msg  = rospy.wait_for_message("/panda/ft/tool", WrenchStamped)
F = np.asarray((msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z))
M = np.asarray((msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z))
# get ground truth, normal is published in world frame it seems
msg_contact = rospy.wait_for_message("/panda/bumper/panda_probe_ball", ContactsState)

# minimize force and torque errors wrt. f_n, f_t1/2, phi, theta
from sympy import *  # apt install python-sympy
theta, phi = symbols('theta phi', real=True)
r = 0.03
L = 0.12 + 0.7

p_coll = Matrix([[r*sin(theta)*cos(phi)],
                [r*sin(theta)*sin(phi)],
                [r*cos(theta)]])
P_hand = Matrix([[0],[0],[-L]])

Normal = Matrix([[sin(theta)*cos(phi)],
                [sin(theta)*sin(phi)],
                [cos(theta)]])

T_1 = 1/r * Matrix([[-r*sin(theta)*sin(phi)],
                [r*sin(theta)*cos(phi)],
                [0]])

T_2 = 1/r * Matrix([[r*cos(theta)*cos(phi)],
                [r*cos(theta)*sin(phi)],
                [r*(-sin(theta))]])
                
# global force balance
f_n, f_t1, f_t2 = symbols('f_n f_t1 f_t2')
F_coll = f_n*(-Normal) + f_t1*T_1 + f_t2*T_2
f_obs_x, f_obs_y, f_obs_z = symbols('f_obs_x f_obs_y f_obs_z')
F_obs = Matrix([[f_obs_x], [f_obs_y], [f_obs_z]])
equ_F = F_coll + F_obs

# global torque balance about contact point
m_obs_x, m_obs_y, m_obs_z = symbols('m_obs_x m_obs_y m_obs_z')
M_obs = Matrix([[m_obs_x], [m_obs_y], [m_obs_z]])
equ_M = M_obs + (P_hand-p_coll).cross(F_obs)
equ_M = simplify(equ_M)

# apt install python-scipy
# does not work, approach to dumb or just false?
x = (f_n, f_t1, f_t2, phi, theta)
lambda_F = lambdify(x, equ_F.subs(f_obs_x, F[0]).subs(f_obs_y, F[1]).subs(f_obs_z, F[2]), "numpy")
lambda_M = lambdify(x, equ_M.subs(m_obs_x, M[0]).subs(m_obs_y, M[1]).subs(m_obs_z, M[2]).subs(f_obs_x, F[0]).subs(f_obs_y, F[1]).subs(f_obs_z, F[2]), "numpy")
x0 = np.array([0.,0.,0.,0.,0.])
def residuum(x):
    return np.sum(lambda_F(*x))+np.sum(lambda_M(*x))
minimize(residuum, x0, method='nelder-mead')

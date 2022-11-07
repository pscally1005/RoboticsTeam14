import general_robotics_toolbox as rox
import numpy as np
import math


def robotJacobian(robot, q0, Rd, Pd, Nmax, alpha, epsilon, tol):
    q0 = (q0*(np.pi/180))[:, None]

    n = q0.shape[0]
    q = np.zeros((n, Nmax+1))
    
    q[:, 0][:, None] = q0
    # for i in range(0, len(q[:,0][:,None])):
    #     q[:,0][:,None][i] = q0[i]

    p0T = np.zeros((3, Nmax+1))
    RPY0T = np.zeros((3, Nmax+1))
    iternum = 0

    TransRP = rox.fwdkin(robot, q0)
    R = TransRP.R
    P = TransRP.p[:, None]
    dR = R @ Rd.T
    dX = np.concatenate(((np.array(rox.R2rpy(R))[None].T), P-Pd))

    while (np.absolute(dX) > tol).any():
        if iternum < Nmax:

            TransRP = rox.fwdkin(robot, q[:, iternum][:, None])

            R = TransRP.R
            p0T[:, iternum][:, None] = TransRP.p[:, None]

            Jq = rox.robotjacobian(robot, q[:, iternum])
            RPY0T[:, iternum][:, None] = (np.array(rox.R2rpy(R))[None].T)

            dR = R @ Rd.T
            dX = np.concatenate(
                ((np.array(rox.R2rpy(dR))[None].T), p0T[:, iternum][:, None]-Pd))

            q[:, iternum+1][:, None] = q[:, iternum][:, None] - \
                alpha*(np.transpose(Jq)@dX)

            iternum = iternum + 1
        else:
            break

    final_q = q[:, :iternum+1]

    return final_q


ex = np . array([1, 0, 0])
ey = np . array([0, 1, 0])
ez = np . array([0, 0, 1])

# link lengths in meters
l0 = 0.061    # base to servo 1
l1 = 0.0435   # servo 1 to servo 2
l2 = 0.08285  # servo 2 to servo 3
l3 = 0.08285  # servo 3 to servo 4
l4 = 0.07385  # servo 4 to servo 5
l5 = 0.05457  # servo 5 to gripper

L1 = l0 + l1
L4 = l4 + l5

# Position vectors
P01 = (l0 + l1)*ez
P12 = np.zeros(3,)
P23 = l2*ex
P34 = -l3*ez
P45 = np.zeros(3,)
P5T = -(l4+l5)*ex

h1 = ez
h2 = -ey
h3 = -ey
h4 = -ey
h5 = -ex

H = np.array([h1, h2, h3, h4, h5]).T
P = np.array([P01, P12, P23, P34, P45, P5T]).T
joint_type = [0]*5

robo = rox.Robot(H=H, P=P, joint_type=joint_type)


R0T = np.zeros((3, 3))  # end effector orientation
P0T = np.zeros((3, 1))  # end effector position
q0 = np.zeros((5, 1))  # initial q guesses

tol = np.array([[0.02, 0.02, 0.02, 0.001, 0.001, 0.001]])
Nmax = 200
epsilon = 0.1
alpha = 0.01

r = robotJacobian(robo, q0, R0T, P0T, Nmax, alpha, epsilon, tol)

from math import prod
import math
import numpy as np
import os
from general_robotics_toolbox import *
import general_robotics_toolbox as rox

# Given rotation matrix
def rotation():
    Rd = np.array([ [0, 0, -1], [0, -1, 0], [-1, 0, 0]])
    print("\nDesired Rotation\n" + str(Rd))
    return Rd

# Given position vector
def position():
    Pd = np.array([ [0], [0], [0.399] ])
    print("\nDesired Position [m]\n" + str(Pd))
    return Pd

# Given joint angles
def joints():
    q0 = np.array([80] * 5)
    print("\nInitial Guess [deg]\n" + str(q0))
    q0 = q0 * math.pi/180
    return q0

# Given tolerance
def tolerance():
    tol = [[0.02, 0.02, 0.02, 0.001, 0.001, 0.001]]
    tol = np.transpose(np.array(tol))
    return tol

# Jacobian inverse
def jacobian_inverse(robot,q0,Rd,Pd,Nmax,alpha,tol):
   
    n = len(q0)
    q = np.zeros((n, Nmax+1))
    q[:,0] = q0
    p0T = np.zeros((3, Nmax+1))
    RPY0T = np.zeros((3, Nmax+1))
    iternum = 0

    H = fwdkin(robot, q[:,0])
    R = H.R
    P = H.p
    P = np.array([[P[0]], [P[1]], [P[2]]])
    dR = np.matmul(R, np.transpose(Rd))
    r = np.array(R2rpy(dR))[None]
    dX = np.concatenate((np.transpose(r), P-Pd))

    while (np.absolute(dX) > tol).any():
        if iternum < Nmax:
            H = fwdkin(robot, q[:,iternum])
            R = H.R
            p0T = H.p
            p0T = np.array([[p0T[0]], [p0T[1]], [p0T[2]]])
            dR = np.matmul(R, np.transpose(Rd))
            r = np.array(R2rpy(dR))[None]
            dX = np.concatenate((np.transpose(r), p0T-Pd))

            Jq = rox.robotjacobian(robot, q[:, iternum])
            j = np.matmul(np.linalg.pinv(Jq), dX)
            q[:, iternum+1] = q[:, iternum] - np.transpose((alpha * j))
            iternum = iternum + 1
        else:
            break

    return q[:, iternum]

# Main function
def main():

    clear = lambda: os.system('clear')
    clear()
    print("------------")
    print("INVERSE KINEMATICS")

    # Given values in problem statement
    Rd = rotation()
    Pd = position()
    q0 = joints()
    tol = tolerance()
    Nmax = 200
    # epsilon = 0.1
    alpha = 0.1

    # Define all the joint lengths [m]
    l0 = 61 * 10**-3
    l1 = 43.5 * 10**-3
    l2 = 82.85 * 10**-3
    l3 = 82.85 * 10**-3
    l4 = 73.85 * 10**-3
    l5 = 54.57 * 10**-3

    x = np.array([1, 0, 0])
    y = np.array([0, 1, 0])
    z = np.array([0, 0, 1])

    # Define the position vectors from i-1 -> i
    P01 = (l0 + l1) * z
    P12 = np.zeros(3)
    P23 = l2 * x
    P34 = -1*l3 * z
    P45 = np.zeros(3)
    P5T = -1*(l4 + l5) * x

    H = np.array([z, -1*y, -1*y, -1*y, -1*x]).T
    P = np.array([P01, P12, P23, P34, P45, P5T]).T
    joint_type = [0,0,0,0,0]
    robot = rox.Robot(H, P, joint_type)
    q = jacobian_inverse(robot,q0,Rd,Pd,Nmax,alpha,tol)
    q = q * 180 / math.pi
    print("\nOutput joint angles [deg]:")
    for i in range(0,len(q)): print(str(q[i])[0:5], end = " ")
    print()
    
    # Htest = fwdkin(robot, q*math.pi/180)
    # print("\n" + str(Htest))
    return q

if __name__ == "__main__" :
    main()
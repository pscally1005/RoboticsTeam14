import numpy as np
import math
import os
import sys
import general_robotics_toolbox as rox

# Define the rotation about the x function for given joint angle q
def rotx(q):
    # Convert joint angle q from degrees -> radians
    q = q * math.pi / 180
    R = [[1, 0, 0], [0, math.cos(q), -1*math.sin(q)], [0, math.sin(q), math.cos(q)]]
    R = np.array(R)
    return R.reshape(3,3)

# Define the rotation about the y function for given joint angle q
def roty(q):
    # Convert joint angle q from degrees -> radians
    q = q * math.pi / 180
    R = [[math.cos(q), 0, math.sin(q)], [0, 1, 0], [-1*math.sin(q), 0, math.cos(q)]]
    R = np.array(R)
    return R.reshape(3,3)

# Define the rotation about the z function for given joint angle q
def rotz(q):
    # Convert joint angle q from degrees -> radians
    q = q * math.pi / 180
    R = [[math.cos(q), -1*math.sin(q), 0], [math.sin(q), math.cos(q), 0], [0, 0, 1]]
    R = np.array(R)
    return R.reshape(3,3)
    
# FORWARD KINEMATICS
def fwdkin(q):
    # Original axes
    ex = np.array([[1], [0], [0]])
    ey = np.array([[0], [1], [0]])
    ez = np.array([[0], [0], [1]])

    # Define the rotation matrices from i-1 -> i
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]

    # Define all i-1 -> i rotations
    R01 = np.array(rotz(q1))
    R12 = np.array(roty(-q2))
    R23 = np.array(roty(-q3))
    R34 = np.array(roty(-q4))
    R45 = np.array(rotx(-q5))
    R5T = np.array(np.identity(3))

    # Define all the joint lengths [m]
    l0 = 61 * 10**-3
    l1 = 43.5 * 10**-3
    l2 = 82.85 * 10**-3
    l3 = 82.85 * 10**-3
    l4 = 73.85 * 10**-3
    l5 = 54.57 * 10**-3

    # Define the position vectors from i-1 -> i
    P01 = (l0 + l1) * ez
    P12 = 0 * ey
    P23 = l2 * ex
    P34 = -1*l3 * ez
    P45 = 0 * ez
    P5T = -1*(l4 + l5) * ex

    # Final rotation matrix 0 -> T
    R02 = np.dot(R01,R12)
    R03 = np.dot(R02,R23)
    R04 = np.dot(R03,R34)
    R05 = np.dot(R04,R45)
    R0T = np.dot(R05,R5T)    

    # Final position vector 0 -> T
    P0T = P01 + np.dot(R01,P12) + np.dot(R02,P23) + np.dot(R03,P34) + np.dot(R04,P45) + np.dot(R05,P5T)
    
    return R0T, P0T

def main():
    clear = lambda: os.system('clear')
    clear()

    print("Enter the 5 joint angles")
    print("Format: q1,q2,q3,q4,q5")
    desired = input()
    q = desired.split(",")
    assert len(q) == 5, "ERROR: Wrong number of joints"
    for i in range(0,len(q)):
        try:
            q[i] = float(q[i])
        except:
            print("ERROR: Joint angles invalid")
            sys.exit(0)
    R, P = fwdkin(q)
    print("\n" + str(R) + "\n\n" + str(P))
    sys.exit(1)

if __name__ == "__main__" :
    main()    
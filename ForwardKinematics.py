from errno import EMSGSIZE
import numpy as np
import math
import os

# Define the 0 fraom x axis
def ex():
    ex = [[1],[0],[0]]
    ex = np.array(ex)
    return ex
    
# Define the 0 fraom y axis
def ey():
    ey = [[0],[1],[0]]
    ey = np.array(ey)
    return ey

# Define the 0 fraom z axis
def ez():
    ez = [[0],[0],[1]]
    ez = np.array(ez)
    return ez

# Define the h vectors for i=1,2,3,4,5,6
def h_axis():
    h1 = ez()
    h2 = ey()
    h3 = ey()
    h4 = ey()
    h5 = ez()
    h6 = ex()

# Define the rotation about the x function for given joint angle q
def rotx(q):
    # Convert joint angle q from degrees -> radians
    q = q * math.pi / 180
    R = [[1, 0, 0], [0, math.cos(q), -1*math.sin(q)], [0, math.sin(q), math.cos(q)]]
    return R

# Define the rotation about the y function for given joint angle q
def roty(q):
    # Convert joint angle q from degrees -> radians
    q = q * math.pi / 180
    R = [[math.cos(q), 0, math.sin(q)], [0, 1, 0], [-1*math.sin(q), 0, math.cos(q)]]
    return R

# Define the rotation about the z function for given joint angle q
def rotz(q):
    # Convert joint angle q from degrees -> radians
    q = q * math.pi / 180
    R = [[math.cos(q), -1*math.sin(q), 0], [math.sin(q), math.cos(q), 0], [0, 0, 1]]
    return R

# Define the 3x3 identity matrix
def identity():
    I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    return I
    
# User input for joint angles
def angles():
    q = [0, 0, 0, 0, 0, 0]
    print("INPUT ROTATION ANGLES FOR ALL 3 SERVOS")
    for i in range (0,6):
        print("\nEnter angle for joint " + str(i+1) + " in degrees [-90,90]")
        q[i] = input()
        valid = False
        while(valid == False) :
            try:
                q[i] = float(q[i])
                if -90 <= q[i] <= 90:
                    valid = True
                else:
                    print("\nERROR: Input is invalid.  Please try again")
                    print("Enter angle for joint " + str(i+1) + " in degrees [-90,90]")
                    q[i] = input()
            except:
                print("\nERROR: Input is invalid.  Please try again")
                print("Enter angle for joint " + str(i+1) + " in degrees [-90,90]")
                q[i] = input()

    print("\nYour joint angles are:")
    for i in range (0,6):
        print("\tJoint " + str(i+1) + ": " + str(q[i]))
    return q


# Main function for program
def main():

    # Clear console screen
    clear = lambda: os.system('clear')
    clear()

    # Define the rotation matrices from i-1 -> i
    q = angles()
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    # Define all i-1 -> i rotations
    R01 = rotz(q1)
    R12 = roty(q2)
    R23 = roty(q3)
    R34 = roty(q4)
    R45 = rotz(q5)
    R56 = rotx(q6)
    R6T = identity()

    # Define all the joint lengths
    l0 = 1
    l1 = 1
    l2 = 1
    l3 = 1
    l4 = 1
    l5 = 1
    l6 = 1

    # Define the position vectors from i-1 -> i
    P01 = (l0 + l1) * ez()
    P12 = 0
    P23 = l2 * ez()
    P34 = l3 * ez()
    P45 = (l4 + l5 + l6) * ez()
    P56 = 0
    P6T = 0

    print("\nSUCCESS")

if __name__ == "__main__" :
    main()
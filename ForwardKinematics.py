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

# Define the h vectors for i=1,2,3,4,5
def h_axis():
    h1 = ez()
    h2 = -1*ey()
    h3 = -1*ey()
    h4 = -1*ey()
    h5 = -1*ez()

# Define the rotation about the x function for given joint angle q
def rotx(q):
    # Convert joint angle q from degrees -> radians
    q = q * math.pi / 180
    R = [[1, 0, 0], [0, math.cos(q), -1*math.sin(q)], [0, math.sin(q), math.cos(q)]]
    R = np.array(R)
    #print(str(R.reshape(3,3)) + "\n")
    return R.reshape(3,3)

# Define the rotation about the y function for given joint angle q
def roty(q):
    # Convert joint angle q from degrees -> radians
    q = q * math.pi / 180
    R = [[math.cos(q), 0, math.sin(q)], [0, 1, 0], [-1*math.sin(q), 0, math.cos(q)]]
    R = np.array(R)
    #print(str(R.reshape(3,3)) + "\n")
    return R.reshape(3,3)

# Define the rotation about the z function for given joint angle q
def rotz(q):
    # Convert joint angle q from degrees -> radians
    q = q * math.pi / 180
    R = [[math.cos(q), -1*math.sin(q), 0], [math.sin(q), math.cos(q), 0], [0, 0, 1]]
    R = np.array(R)
    #print(str(R.reshape(3,3)) + "\n")
    return R.reshape(3,3)

# Define the 3x3 identity matrix
def identity():
    I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    return I
    
# User input for joint angles
def angles():
    q = [0, 0, 0, 0, 0, 0]
    for i in range (0, len(q)-1):
        print("\nEnter angle for joint " + str(i+1) + " in degrees [0, 180]")
        q[i] = input()
        valid = False
        while(valid == False) :
            try:
                q[i] = float(q[i])
                if 0 <= q[i] <= 180:
                    valid = True
                else:
                    print("\nERROR: Input is invalid.  Please try again")
                    print("Enter angle for joint " + str(i+1) + " in degrees [0, 180]")
                    q[i] = input()
            except:
                print("\nERROR: Input is invalid.  Please try again")
                print("Enter angle for joint " + str(i+1) + " in degrees [0, 180]")
                q[i] = input()

    print("\nYour joint angles are: [deg]\n")
    for i in range (0, len(q)-1):
        print("\tJoint " + str(i+1) + ": " + str(q[i]))
    return q

# Form the homogenous transform with R0T and P0T
def homogTransf(R0T, P0T):
    H = np.zeros((len(R0T)+1, len(R0T)+1))
    for i in range(0, len(R0T)+1):
        for j in range(0, len(R0T[0])+1):
            if i < len(R0T) and j < len(R0T[0]):
                H[i][j] = R0T[i][j]
            elif j < len(R0T[0]):
                H[i][j] = 0
            elif i < len(R0T):
                H[i][j] = P0T[i][0]
            else :
                H[i][j] = 1
    
    H = np.array(H)
    H = H.reshape(len(R0T)+1, len(R0T)+1)
    return H

# Main function for program
def main():

    # Clear console screen
    clear = lambda: os.system('clear')
    clear()
    print("------------")
    print("FORWARD KINEMATICS")

    # Define the rotation matrices from i-1 -> i
    q = angles()
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    # Define all i-1 -> i rotations
    R01 = np.array(rotz(q1))
    R12 = np.array(roty(-q2))
    R23 = np.array(roty(-q3))
    R34 = np.array(roty(-q4))
    R45 = np.array(rotx(-q5))
    R5T = np.array(identity())

    # Define all the joint lengths [mm]
    l0 = 61
    l1 = 43.5
    l2 = 82.85
    l3 = 82.85
    l4 = 73.85
    l5 = 54.57

    # Define the position vectors from i-1 -> i
    P01 = (l0 + l1) * ez()
    P12 = 0 * ey()
    P23 = l2 * ex()
    P34 = -1*l3 * ez()
    P45 = 0 * ez()
    P5T = -1*(l4 + l5) * ex()

    # Final rotation matrix 0 -> T
    R02 = np.dot(R01,R12)
    R03 = np.dot(R02,R23)
    R04 = np.dot(R03,R34)
    R05 = np.dot(R04,R45)
    R0T = np.dot(R05,R5T)    

    # Final position vector 0 -> T
    P0T = P01 + np.dot(R01,P12) + np.dot(R02,P23) + np.dot(R03,P34) + np.dot(R04,P45) + np.dot(R05,P5T)

    # Print final rotation
    print("\nThe final rotation R0T is: \n")# + str(R0T.reshape(3,3)))
    # Round R0T to 3 decimal places
    for i in range(0, len(R0T)):
        for j in range(0, len(R0T[i])):
            if str(format(R0T[i][j], '.3f')) == "-0.000":
                R0T[i][j] = 0
            print("\t" + str(format(R0T[i][j], '.3f')), end = "\t")
        print()

    # Print final position
    print("\nThe final position P0T is: [mm]\n")# + str(P0T.reshape(3,1)))
    # Round P0T to 3 decimal places
    for i in range(0, len(P0T)):
        if P0T[i] == -0:
            P0T[i] = 0
        print("\t" + str(format(P0T[i][0], '.3f')))

    # Print homogenous transform H0T
    print("\nThe homogeneous transform H0T is:\n")
    H = homogTransf(R0T, P0T)
    # Round H0T to 3 decimal places
    for i in range(0, len(H)):
        for j in range(0, len(H[i])):
            print("\t" + str(format(H[i][j], '.3f')), end = "\t")
        print()


    # End of Code
    print("\n------------")

if __name__ == "__main__" :
    main()
import math as mt
import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt


"""
Forward Kinematic and Inverse kinematic of 3-axis robot manipulator
"""

def inverse(theta, ep):  # inverse kinematic

    Link = np.array([10, 10, 10])

    theta[0] = mt.atan2(ep[1], ep[0])  # angle of jt1

    D = np.linalg.norm(ep - [0, 0, Link[0]])
    #D = (ep[0] ** 2 + ep[1] ** 2 + (ep[2] - Link[0]) ** 2 - (Link[1] ** 2 + Link[2] ** 2)) / (2 * Link[1] * Link[2])

    theta[2] = mt.acos((Link[1]**2 + Link[2]**2 - D**2) / (2*Link[1]*Link[2])) - mt.pi
    #theta[2] = mt.atan2((1 - D**2)**0.5, D)  # angle of jt3

    #print(theta[2])
 
    theta[1] = mt.atan2(ep[2] - Link[0], (ep[0]**2 + ep[1]**2)**0.5) - mt.atan2(Link[2]*mt.sin(theta[2]), Link[1] + Link[2]*mt.cos(theta[2]))  # angle of jt2

    #theta = [x * 180 / mt.pi for x in theta]

    return theta

def forward(jt, theta2):  # forward kinematics

    DH = np.array([[0, 1.570796327, 10, theta2[0]],
                   [10, 0, 0, theta2[1]],
                   [10, 0, 0, theta2[2]]])

    temp = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    for i in range(0, jt+1):

        A = np.array([[mt.cos(DH[i, 3]), -mt.sin(DH[i, 3])*mt.cos(DH[i, 1]), mt.sin(DH[i, 3])*mt.sin(DH[i, 1]),
                       DH[i, 0]*mt.cos(DH[i, 3])],
                      [mt.sin(DH[i, 3]), mt.cos(DH[i, 3])*mt.cos(DH[i, 1]), -mt.cos(DH[i, 3])*mt.sin(DH[i, 1]),
                       DH[i, 0]*mt.sin(DH[i, 3])],
                      [0, mt.sin(DH[i, 1]), mt.cos(DH[i, 1]), DH[i, 2]],
                      [0, 0, 0, 1]])
                      
        temp = temp.dot(A)

        T = np.array([temp[0, 3], temp[1, 3], temp[2, 3]])

    return T



"""
To print the Information:

i. input target position 

ii. output angle inverse kinematic

iii. output target position of forward kinematic 
"""

tp = np.array([13, -10, 10]) # modify your target position at here

th = [0, 0, 0]

print("\ntarget point:", tp, "(unit:cm)\n")  # i


print("ik angle:", inverse(th, tp), "(unit: degree)\n")  # ii

fk = forward(2, inverse(th, tp))

#o = [fk[0, 3], fk[1, 3], fk[2, 3]]

print("fk tp:", fk, "\n")  # iii


"""
Plotting:

The end position (target position) and three joints of 3-jt-robotic arm
"""

fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')

x = [0, forward(0, inverse(th, tp))[0], forward(1, inverse(th, tp))[0], forward(2, inverse(th, tp))[0]]
y = [0, forward(0, inverse(th, tp))[1], forward(1, inverse(th, tp))[1], forward(2, inverse(th, tp))[1]]
z = [0, forward(0, inverse(th, tp))[2], forward(1, inverse(th, tp))[2], forward(2, inverse(th, tp))[2]]

ax.scatter(x, y, z, c='b', marker='o', linewidths=4.0)

ax.plot(x, y, z, c='g', linewidth=2.0)

ax.set_xlabel('X (unit:cm)')
ax.set_ylabel('Y (unit:cm)')
ax.set_zlabel('Z (unit:cm)')

ax.set_xlim(-10, 25)
ax.set_ylim(-25, 25)
ax.set_zlim(-0, 25)

plt.show()
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 19 14:46:52 2022

@author: Greg McDowell
"""

# %%

import numpy as np

# Known lengths of arm

a1 = 5.0
a2 = 5.0

X = 0.0
Y = 8.0
Z = 6.0

Theta2 = np.arctan2(Y,X)

# Below is the geometrically derived inverse kinematics for the 3DOF arm with two prismatic and one revolute joints

print('Theta2 =', Theta2*180/3.1415)

d3 = np.sqrt(np.square(X) + np.square(Y)) - a2

print('d3 = ', d3)

d1 = Z - a1

print('d1 = ', d1)

# Visualize in matplotlib

# %%

%matplotlib
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure() # create new figure
ax = fig.add_subplot(projection='3d') # add 3d plot

# make and show plot
ax.plot([0, 0], [0, 0], [0, 10], color='k')  # extend in z direction
ax.plot([0, 0], [0, 10], [0, 0], color='k')   # x axis
ax.plot([0, 10], [0, 0], [0, 0], color='k')   # y axis

# Position of arm
xa = np.cos(Theta2)*(d3+a2)
ya = np.sin(Theta2)*(d3+a2)
za = d1+a1

# Link 1 vectors
x1 = [0, np.cos(Theta2)*(d3+a2)]
y1 = [0, np.sin(Theta2)*(d3+a2)]
z1 = [d1+a1, d1+a1]

# Link 2 vectors
x2 = [0, 0]
y2 = [0, 0]
z2 = [0, d1+a1]

ax.plot(x1, y1, z1)    # extend in xy plane with link 2
ax.plot(x2, y2, z2)    # extend in xy plane with link 3
ax.scatter(xa, ya, za, s=4, c='b')

# Now we have derived R0_3 in the report, we can use the columns of the matrix
# to find the unit vectors of that frame

R0_3 = [[np.sin(Theta2), 0.0, np.cos(Theta2)],
    [-np.cos(Theta2),0.0,np.sin(Theta2)],
    [0.0,-1.0,0.0]]

# But this is not nessecarily the orientation of the final spherical wrist, because 
# it has rotations itself. To be able to draw the starting orientation of the 
# gripper on the graph we need to figure out what the orientation of the gripper would 
# be if all angles in the spherical wrist were zero.

# Create R3_6 forward kinematics for thetas all zero, then use R0_3 dot product to find R0_6, 
# this will then show where frame is relative to x-y axis, then can consider if the angles suggested
# for theta 4 5 6 are correct.

theta4 = 0.0*3.1415/180
 
R3_4_FK = [[np.cos(theta4), -np.sin(theta4),0], 
           [np.sin(theta4), np.cos(theta4), 0], 
           [0,0,1]]

invR0_3 = np.linalg.inv(R0_3)

# Chosen R0_4
R0_4 = [[0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0]]

R3_4 = np.dot(invR0_3, R0_4)

# Then with R3_6 and the DH dervived equations for the spherical wrist we can find the 
# angles 4 5 and 6

theta4 = np.arccos(R3_4[0][0])
print('Theta4 =', theta4*180/3.1415)
theta4 = np.arcsin(R3_4[1][0])
print('Theta4 =', theta4*180/3.1415)

# %%

# Uncomment these lines below depending on which orientations you want to view

# Show R0_3 orientation

# ax.plot([xa, xa+R0_3[0][0]],[ya, ya+R0_3[1][0]],[za, za+R0_3[2][0]], color='b')
# ax.plot([xa, xa+R0_3[0][1]],[ya, ya+R0_3[1][1]],[za, za+R0_3[2][1]], color='g')
# ax.plot([xa, xa+R0_3[0][2]],[ya, ya+R0_3[1][2]],[za, za+R0_3[2][2]], color='c')

# Show calculated R0_4

R0_4 = np.dot(R0_3, R3_4)

# # # Add frame of gripper to plot the R0_4 frame
ax.plot([xa, xa+R0_4[0][0]],[ya, ya+R0_4[1][0]],[za, za+R0_4[2][0]], color='b')
ax.plot([xa, xa+R0_4[0][1]],[ya, ya+R0_4[1][1]],[za, za+R0_4[2][1]], color='g')
ax.plot([xa, xa+R0_4[0][2]],[ya, ya+R0_4[1][2]],[za, za+R0_4[2][2]], color='c')

# # Show what R0_4 would be with zero angles

R0_4_zero = np.dot(R0_3, R3_4_FK)

# Add frame of gripper to plot the R0_6 frame

# ax.plot([xa, xa+R0_4_zero[0][0]],[ya, ya+R0_4_zero[1][0]],[za, za+R0_4_zero[2][0]], color='b')
# ax.plot([xa, xa+R0_4_zero[0][1]],[ya, ya+R0_4_zero[1][1]],[za, za+R0_4_zero[2][1]], color='g')
# ax.plot([xa, xa+R0_4_zero[0][2]],[ya, ya+R0_4_zero[1][2]],[za, za+R0_4_zero[2][2]], color='c')

plt.show()

# %%

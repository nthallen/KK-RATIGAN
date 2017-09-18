#Tests for the eventual implementation of POPS and LIDAR
#Miles Allen
#13 September 2017
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle, PathPatch
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.pyplot as plt
import numpy as np
import math

# This method determines whether the given point is inside the cloud or not.
def is_in_cloud(x,y,z):
    distance = math.sqrt((x-0)**2 + (z-50)**2)
    if distance <= 50:
        if (0 <= y <= 1500):
            return True
        else:
            return False
    return False

# This method draws a line given O, the origin point, and D, the vector/direction.
def lidar_line(O,D):
    pass
    #vdot(a, b) 	Return the dot product of two vectors.

# This method draws a wire-frame of the cloud.
def display_cloud(lines,ax):
    #radius_line = art3d.Line3D((0,0),(0,1500),(50,50),ls='--')
    #radius_line.set_pickradius(50)
    #ax.add_line(radius_line)

    # Add circle at y=0
    p = Circle((0,50),50)
    ax.add_patch(p)
    art3d.pathpatch_2d_to_3d(p,z=0,zdir='y')

    #Add circle at y=1500
    q = Circle((0,50),50)
    ax.add_patch(q)
    art3d.pathpatch_2d_to_3d(q,z=1500,zdir='y')

    #Generate wireframe
    a = 360/lines
    rad = math.radians(a)
    accumulating_rad = rad
    for i in range(lines):
        x = 50 * math.cos(accumulating_rad)
        z = 50 * math.sin(accumulating_rad) + 50
        new_line = art3d.Line3D((x,x),(0,1500),(z,z),ls='--')
        ax.add_line(new_line)
        accumulating_rad += rad

# Main method.
def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    #Set graph limits
    ax.set_xlim(-200,200)
    ax.set_ylim(-10,1510)
    ax.set_zlim(-100,200)

    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')

    display_cloud(16,ax)
    if (is_in_cloud(250,750,50)):
        print("It's in the cloud")
    else:
        print("It isn't in the cloud")
    plt.show()
main()

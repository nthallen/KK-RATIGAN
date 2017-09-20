import math
import numpy
from mayavi.mlab import *

# This method takes in a number of lines to generate and displays the cloud.
def display_cloud(divisions):
    a = 360/divisions
    rad = math.radians(a)
    accumulating_rad = rad
    for i in range(divisions):
        x = 50 * math.cos(accumulating_rad)
        z = 50 * math.sin(accumulating_rad) + 50
        line = plot3d
        accumulating_rad += rad
        line = plot3d([x,x],[0,1500],[z,z],tube_radius=0.5)

# Main method.
def main():
    display_cloud(16)
    #axes(xlabel="x axis",ylabel="y axis",zlabel="z axis")
    show()
main()

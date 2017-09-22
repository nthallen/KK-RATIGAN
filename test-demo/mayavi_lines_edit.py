import math
import time
import numpy as np
from mayavi import mlab

RESOLUTION=100

# This method determines whether the given point is inside the cloud or not.
def is_in_cloud(position):
    x,y,z=position
    distance = math.sqrt((x-0)**2 + (z-0)**2)
    if distance <= 50:
        if (0 <= y <= 1500):
            return True
        else:
            return False
    return False

# This method displays cloud lines with adjustments to account for wind movement.
def display_cloud(divisions,a=0,b=0,c=0):
    a=360/divisions
    rad=math.radians(a)
    accumulating_rad=rad
    for i in range(divisions):
        x=50*math.cos(accumulating_rad)
        z=50*math.sin(accumulating_rad)
        accumulating_rad+=rad
        mlab.plot3d([x+a,x+a],[0+b,1500+b],[z+c,z+c],tube_radius=0.5,color=(1,1,1))

# This method sets the camera's initial view.
def setup_view():
    mlab.view(azimuth=270,elevation=90,distance=50,focalpoint=(0,1400,0))

# This method moves the camera along the flight path.
def pan():
    time.sleep(1)
    mlab.move(forward=150,right=0,up=0) # For no reason
    time.sleep(1)
    mlab.move(forward=50,right=50,up=0) # Move 50 meters away from the cloud
    time.sleep(1)
    mlab.move(forward=50,right=50,up=0) # Turn back around
    mlab.yaw(180)

# This method draws a line where the LIDAR instrument is pointing.
def lidar_line(azimuth,elevation,position):
    x1,y1,z1=position
    azimuth_matrix=[[math.cos(azimuth),-math.sin(azimuth),0],[math.sin(azimuth),math.cos(azimuth),0],[0,0,1]]
    elevation_matrix=[[1,0,0],[0,math.cos(elevation),math.sin(elevation)],[0,-math.sin(elevation),math.cos(elevation)]]
    dot=np.matmul(elevation_matrix,azimuth_matrix)
    x2,y2,z2=np.matmul(position,dot)
    
    bin_dist=np.mgrid[100:15000:(RESOLUTION*1j)]
    
    mlab.plot3d([x1,x2],[y1,y2],[z1,z2],tube_radius=1,color=(1,0,0))

# This method creates the mesh of the cloud.
def create_mesh(radius=50):
    deg=np.mgrid[0:360:((16+1)*1j)];
    rad=np.radians(deg)
    yy=(0,1500)
    y,r=np.meshgrid(yy,rad)
    x=radius*np.sin(r)
    z=radius*np.cos(r)
    mlab.mesh(x,y,z,color=(1,1,1),opacity=0.5)

# Main method.
def main():
    fig=mlab.figure(bgcolor=(0.52,0.8,1))
    create_mesh()
    lidar_line(180,45,(400,400,-400))
    mlab.show()
main()
# Miles E. Allen, 12 October 2017

import interactions
import simulation
import bitvector
import math
import numpy as np
from mayavi import mlab

RESOLUTION=100
#(0.52,0.8,0.92) # Sky Blue Color

# This method determines whether the given point is inside the cloud or not.
def is_in_cloud(position):
    x,y,z=position
    distance = math.sqrt(x**2 + z**2)
    if distance <= 50:
        if (0 <= y <= 1500):
            return 1
        else:
            return 0
    return 0

# A class to hold information regarding the lidar and its lines.
class Lidar():
    lidar_state_queue=None
    sphere_state_queue=None
    max_size=None
    lidar_azimuth=None
    lidar_elevation=None
    reference_direction=[0,1,0]
    h_seq=None
    v_seq=None
    hv_seq=None
    seq=None
    off=None

    def __init__(self,maxsize):
        self.max_size=maxsize
        self.lidar_state_queue=interactions.IndexableQueue(maxsize=self.max_size)
        self.sphere_state_queue=interactions.IndexableQueue(maxsize=self.max_size)
        self.lidar_azimuth=0
        self.lidar_elevation=0
        self.h_seq=bitvector.BitSequenceHorizontal(4)
        self.v_seq=bitvector.BitSequenceVertical(4)
        self.hv_seq=bitvector.BitSequence(4,3)
        self.seq=self.hv_seq
        self.off=False

    def mode_select(self,mode):
        if (mode=="LIDAR Multi Scan"):
            self.off=False
            self.seq=self.hv_seq
        if (mode == "LIDAR Horizontal Scan"):
            self.off=False
            self.seq=self.h_seq
        if (mode == "LIDAR Vertical Scan"):
            self.off=False
            self.seq=self.v_seq
        if (mode == "LIDAR OFF"):
            self.off=True

    # This method calculates the direction in which the LIDAR should be facing.
    def lidar_direction(self,azimuth,elevation):
        gondola_az_ma=simulation.azimuth_matrix(azimuth)
        gondola_el_ma=simulation.elevation_matrix(elevation)
        lidar_az_ma=simulation.azimuth_matrix(self.lidar_azimuth)
        lidar_el_ma=simulation.elevation_matrix(self.lidar_elevation)
        step_1=np.matmul(gondola_el_ma,gondola_az_ma)
        step_2=np.matmul(lidar_el_ma,lidar_az_ma)
        step_3=np.matmul(step_2,step_1)
        return np.matmul(self.reference_direction,step_3)

    # A method to perform several lidar scans.
    def scan(self,max_angle):
        degrees_horizontal=max_angle
        degrees_vertical=max_angle/2
        az_el=self.seq.evaluate(degrees_horizontal,degrees_vertical)
        azimuth=az_el[0]
        elevation=az_el[1]
        self.lidar_azimuth=azimuth
        self.lidar_elevation=elevation

    # This method draws a line where the LIDAR instrument is pointing.
    def lidar_line(self,azimuth,elevation,position):
        x1,y1,z1=position
        x2,y2,z2=self.lidar_direction(azimuth,elevation)
        bin_dist=np.mgrid[100:2000:(RESOLUTION*1j)]
        x = x1 + bin_dist*x2
        y = y1 + bin_dist*y2
        z = z1 + bin_dist*z2
        t = 0*bin_dist
        for i in range(len(bin_dist)):
            t[i] = is_in_cloud((x[i],y[i],z[i]))
        if (self.lidar_state_queue.qsize()<self.max_size):
            if (self.off==False):
                new_line=mlab.plot3d(x,y,z,t,tube_radius=1,reset_zoom=False,colormap='Greys')
                ms_line=new_line
                self.lidar_state_queue.put(ms_line)
            else:
                new_line=mlab.plot3d(x,y,z,t,tube_radius=1,reset_zoom=False,colormap='Greys',opacity=0)
                ms_line=new_line
                self.lidar_state_queue.put(ms_line)
            new_sphere=mlab.points3d(x1,y1,z1,reset_zoom=False,color=(1,1,1))
            ms_sphere=new_sphere.mlab_source
            self.sphere_state_queue.put(ms_sphere)
        else:
            if (self.off==False):
                old_line=self.lidar_state_queue.get()
                old_line.actor.property.opacity=1
                old_line.mlab_source.set(x=x,y=y,z=z,scalars=t,tube_radius=1,reset_zoom=False,colormap='Greys')
                self.lidar_state_queue.put(old_line)
            else:
                old_line=self.lidar_state_queue.get()
                old_line.actor.property.opacity=0
                #old_line.set(x=x,y=y,z=z,scalars=t,tube_radius=1,reset_zoom=False,colormap='Greys')
                self.lidar_state_queue.put(old_line)
            old_sphere=self.sphere_state_queue.get()
            old_sphere.set(x=x1,y=y1,z=z1,reset_zoom=False,color=(1,1,1))
            self.sphere_state_queue.put(old_sphere)
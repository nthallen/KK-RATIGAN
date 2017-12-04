# Miles E. Allen, 12 October 2017

import interactions
import simulation
import bitvector
import math
import numpy as np
from mayavi import mlab
import matplotlib.pyplot as plt

RESOLUTION=20
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

# A Class to hold LIDAR graphical and histogram information.
class Lidar_Queue_Item():
    lidar_line=None
    lidar_output=None
    
    def __init__(self,line,output):
        self.lidar_line=line
        self.lidar_output=output

# A class to hold information regarding the lidar and its lines.
class Lidar():
    gondola=None
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
    max_angle=None
    vmin=None
    vmax=None
    lidar_vmin=None
    lidar_vmax=None
    fig=None
    ax=None
    lidar_output=None
    plot=None
    old_fig=None
    manual_max_override=False

    # Initialization method.
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
        self.max_angle=23
        self.vmin=1
        self.vmax=1
        self.lidar_vmin=1
        self.lidar_vmax=1

    # This method sets up the graph when its window is told to open.
    def handle_openings(self):
        self.fig=plt.figure()
        self.plot=plt
        plt.ion()
        self.fig.canvas.mpl_connect('close_event', self.handle_close)
        self.fig.canvas.mpl_connect('button_press_event', self.handle_click)

    # This method assures that the proper booleans are set when the graph window closes.
    def handle_close(self,evt):
        self.gondola.graph_on=False

    # This method maps vmin and vmax to mouse click locations.
    def handle_click(self,evt):
        x_pos=evt.xdata
        #left_dist=math.fabs(x_pos-self.vmin)
        #right_dist=math.fabs(self.vmax-x_pos)
        self.vmin=x_pos
        self.plot.axvline(self.vmin,color=(1,0,0))
        print("   >>> new vmin ::",self.vmin)
        #if (left_dist < right_dist):
        #    self.vmin=x_pos
        #    self.plot.axvline(self.vmin,color=(1,0,0))
        #    print("   >>> new vmin ::",self.vmin)
        #else:
        #    self.vmax=x_pos
        #    self.plot.axvline(self.vmax,color=(0,1,0))
        #    print("   >>> new vmax ::",self.vmax)
        self.manual_max_override=True

    # This method graphs the results of the LIDAR retrieval.
    def graph_lidar_results(self):
        self.plot.clf()
        self.fig.suptitle('LIDAR Retrieval', fontsize=14, fontweight='bold')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel('counts')
        self.ax.set_ylabel('bin limits')
        total_range=self.lidar_vmax-self.lidar_vmin
        bins=30
        bin_length=total_range/bins
        self.ax.set_xlim([0,(self.lidar_vmax)])
        self.ax.set_ylim([0,bins])
        
        bin_centers=(np.mgrid[0:bins+1:1]-0.5)*bin_length
        bin_counts=[0]*(bins+1)
        #self.plot.axvline(self.vmax,color=(0,1,0))
        if (self.vmin==None):
            self.vmin=0
        self.plot.axvline(self.vmin,color=(1,0,0))
        for queue_item in self.lidar_state_queue:
            output=queue_item.lidar_output
            for f_element in output:
                element=f_element-self.vmin
                element=math.floor(element/bin_length)
                #if (element < len(bin_counts)):
                bin_counts[element]=bin_counts[element]+1
        self.ax.plot(bin_centers,bin_counts)
        #for i in range(len(bin_counts)-1, -1, -1):
            #if bin_counts[i]>0:
                #answer=i+(bin_length/2)
                #if answer>=6:
                    #self.vmax=answer
        self.plot.pause(0.05)

    # This method provides values for the scalars of the lidar's line.
    def lidar_retrieval(self,x,y,z,x_0,y_0,z_0,bin_length,azimuth):
        dist=np.sqrt(np.power(x,2)+np.power(z,2))
        lidar_range=np.sqrt(np.power(x-x_0,2)+np.power(y-y_0,2)+np.power(z-z_0,2))
        beta=1e6
        cloud_radius=50
        cloud_max_density=5
        background=0
        density_1=cloud_max_density*np.exp(-(1/2)*np.power((dist/cloud_radius),2))
        density=density_1+background
        signal=(beta*bin_length*density)/np.power(lidar_range,2)
        SN=0*signal
        for i in range(1,len(signal)):
            if (y[i] >= 0) and (y[i] <= 1500):
                SN[i]=np.random.poisson(signal[i])
            else:
                SN[i] = 0
        output=SN*np.power(lidar_range,2)/(beta*bin_length)
        if not(self.manual_max_override):
            for element in output:
                if element < self.lidar_vmin:
                    self.lidar_vmin=element
                if element > self.lidar_vmax:
                    self.lidar_vmax=element
                    self.vmax=element
        self.lidar_output=output
        return output

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
    def scan(self):
        degrees_horizontal=self.max_angle
        degrees_vertical=self.max_angle/2
        az_el=self.seq.evaluate(degrees_horizontal,degrees_vertical)
        azimuth=az_el[0]
        elevation=az_el[1]
        self.lidar_azimuth=azimuth
        self.lidar_elevation=elevation

    # This method draws a line where the LIDAR instrument is pointing.
    def lidar_line(self,azimuth,elevation,position):
        x1,y1,z1=position
        x2,y2,z2=self.lidar_direction(azimuth,elevation)
        bin_dist=np.mgrid[100:2000:RESOLUTION]
        x = x1 + bin_dist*x2
        y = y1 + bin_dist*y2
        z = z1 + bin_dist*z2
        t=self.lidar_retrieval(x,y,z,x1,y1,z1,RESOLUTION,azimuth)
        if (self.lidar_state_queue.qsize()<self.max_size):
            if (self.off==False):
                new_line=mlab.plot3d(x,y,z,t,tube_radius=1,reset_zoom=False,colormap='Reds',vmin=self.vmin,vmax=self.vmax)
                new_queue_item=Lidar_Queue_Item(new_line,t)
                self.lidar_state_queue.put(new_queue_item)
            else:
                new_line=mlab.plot3d(x,y,z,t,tube_radius=1,reset_zoom=False,colormap='Reds',opacity=0)
                new_queue_item=Lidar_Queue_Item(new_line,t)
                self.lidar_state_queue.put(new_queue_item)
            new_sphere=mlab.points3d(x1,y1,z1,reset_zoom=False,color=(1,1,1))
            ms_sphere=new_sphere.mlab_source
            self.sphere_state_queue.put(ms_sphere)
        else:
            if (self.off==False):
                old_queue_item=self.lidar_state_queue.get()
                old_line=old_queue_item.lidar_line
                old_line.actor.property.opacity=1
                old_line.mlab_source.set(x=x,y=y,z=z,scalars=t,tube_radius=1,reset_zoom=False,colormap='Reds',vmin=self.vmin,vmax=self.vmax)
                old_queue_item.line=old_line
                self.lidar_state_queue.put(old_queue_item)
            else:
                old_queue_item=self.lidar_state_queue.get()
                old_line=old_queue_item.lidar_line
                old_line.actor.property.opacity=0
                old_queue_item.lidar_line=old_line
                #old_line.set(x=x,y=y,z=z,scalars=t,tube_radius=1,reset_zoom=False,colormap='Greys')
                self.lidar_state_queue.put(old_queue_item)
            old_sphere=self.sphere_state_queue.get()
            old_sphere.set(x=x1,y=y1,z=z1,reset_zoom=False,color=(1,1,1))
            self.sphere_state_queue.put(old_sphere)
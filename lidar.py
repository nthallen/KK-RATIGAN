# Miles E. Allen, 12 October 2017

import interactions
import simulation
import bitvector
import math
import numpy as np
from mayavi import mlab
import matplotlib.pyplot as plt

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
    lidar_scan_azimuth=None
    lidar_elevation=None
    lidar_scan_elevation=None
    reference_direction=[0,1,0]
    h_seq=None
    v_seq=None
    hv_seq=None
    seq=None
    off=True
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
    lidar_resolution=None
    new_cloud=None
    range_has_changed=False
    output_output_file=None

    # Initialization method.
    def __init__(self,maxsize,resolution):
        self.max_size=maxsize
        self.lidar_state_queue=interactions.IndexableQueue(maxsize=self.max_size)
        self.sphere_state_queue=interactions.IndexableQueue(maxsize=self.max_size)
        self.lidar_azimuth=0
        self.lidar_scan_azimuth=0
        self.lidar_elevation=0
        self.lidar_scan_elevation=0
        self.h_seq=bitvector.BitSequenceHorizontal(4)
        self.v_seq=bitvector.BitSequenceVertical(4)
        self.hv_seq=bitvector.BitSequence(4,3)
        self.seq=self.hv_seq
        self.off=True
        self.max_angle=23
        self.vmin=0
        self.vmax=2.5
        self.lidar_vmin=0
        self.lidar_vmax=2.5
        self.lidar_resolution=resolution
        self.range_has_changed=False
        self.output_output_file = open('output_output.txt', 'w')
        self.output_output_file.write(">OUTPUT OUTPUT\n")

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
        if bin_length==0:
            bin_length=1
        self.ax.set_xlim([0,(self.lidar_vmax)])
        self.ax.set_ylim([0,bins])
        
        bin_centers=(np.mgrid[0:bins+1:1]-0.5)*bin_length
        bin_counts=[0]*(bins+1)
        if (self.vmin==None):
            self.vmin=0
        self.plot.axvline(self.vmin,color=(1,0,0))
        for queue_item in self.lidar_state_queue:
            output=queue_item.lidar_output
            #print("graph_lidar_results output",output)
            for f_element in output:
                if (f_element<self.vmin):
                    element=0
                elif (f_element>self.lidar_vmax):
                    element=bins
                else:
                    element=f_element-self.vmin
                    element=math.floor(f_element/bin_length)
                element=f_element-self.vmin
                element=math.floor(element/bin_length)
                if (element < len(bin_counts)):
                    bin_counts[element]=bin_counts[element]+1
        #print(bin_counts)
        self.ax.plot(bin_centers,bin_counts)
        self.plot.pause(0.05)

    # This method provides values for the scalars of the lidar's line.
    def lidar_retrieval(self,x,y,z,lidar_range):
        # x,y,z are the vectors defining the centerpoint of the LIDAR bins.
        # lidar_range is equal to bin_dist
        
        #dist=np.sqrt(np.power(x,2)+np.power(z,2))
        beta=1e6
        #cloud_radius=50
        #cloud_max_density=5
        
        background=0
        #density_1=cloud_max_density*np.exp(-(1/2)*np.power((dist/cloud_radius),2))
        density_1=np.zeros(np.shape(x))
        for i in range(len(x)):
            density_1[i]=self.new_cloud.check_cell(np.array((x[i],y[i],z[i])),i,lidar_range[i])
        density=density_1+background
        signal=(beta*self.lidar_resolution*density)/np.power(lidar_range,2)
        SN=0*signal
        for i in range(len(signal)):
            if (y[i] >= 0) and (y[i] <= 1500):
                SN[i]=np.random.poisson(signal[i])
            else:
                SN[i] = 0
        output=SN*np.power(lidar_range,2)/(beta*self.lidar_resolution)
        output_printing_string = ", "+str(output)
        self.output_output_file.write(output_printing_string)
        print_string = "   >>>LIDAR_OUTPUT[0]:"+str(output[0])+"\n"
        self.new_cloud.cloud_output_file.write(print_string)
        if not(self.manual_max_override):
            for element in output:
                if element < self.lidar_vmin:
                    self.lidar_vmin=element
                if element > self.lidar_vmax:
                    self.lidar_vmax=element
                    self.vmax=element
                    self.range_has_changed=True
        #print("self.lidar_vmax",self.lidar_vmax)
        self.lidar_output=output
        #print("lidar_retrieval output",output)
        return output

    # This method calculates the direction in which the LIDAR should be facing.
    def lidar_direction(self,azimuth,elevation):
        gondola_az_ma=simulation.azimuth_matrix(azimuth)
        gondola_el_ma=simulation.elevation_matrix(elevation)
        lidar_az_ma=simulation.azimuth_matrix(self.lidar_scan_azimuth)
        lidar_el_ma=simulation.elevation_matrix(self.lidar_scan_elevation)
        step_1=np.matmul(gondola_el_ma,gondola_az_ma)
        step_2=np.matmul(lidar_el_ma,lidar_az_ma)
        step_3=np.matmul(step_2,step_1)
        return np.matmul(self.reference_direction,step_3)

    # A method to perform several lidar scans.
    def scan(self):
        degrees_horizontal=self.max_angle
        degrees_vertical=self.max_angle/2
        az_el=self.seq.evaluate(degrees_horizontal,degrees_vertical)
        self.lidar_scan_azimuth=az_el[0]
        self.lidar_scan_elevation=az_el[1]

    # This method draws a line where the LIDAR instrument is pointing.
    def lidar_line(self,azimuth,elevation,position):
        x1,y1,z1=position
        x2,y2,z2=self.lidar_direction(azimuth,elevation)
        bin_dist=np.mgrid[100:2000:self.lidar_resolution]
        x = x1 + bin_dist*x2
        y = y1 + bin_dist*y2
        z = z1 + bin_dist*z2
        if not(self.off):
            t=self.lidar_retrieval(x,y,z,bin_dist)
        else:
            t=[0]*z
        
        #for i in range(95):
        #    j=95-i
        #    t[i]=j/50.0
        
        print_string = "   >>>vmin("+str(self.vmin)+"), vmax("+str(self.vmax)+")\n"
        self.new_cloud.cloud_output_file.write(print_string)
        #if (self.range_has_changed):
        #    for element in self.lidar_state_queue:
        #        old_line=element.lidar_line
        #        old_line.mlab_source.set(vmin=self.vmin,vmax=self.lidar_vmax)
        #    self.range_has_changed=False
        if (self.lidar_state_queue.qsize()<self.max_size):
            if not(self.off):
                new_line=mlab.plot3d(x,y,z,t,tube_radius=1,reset_zoom=False,colormap='Reds',vmin=self.vmin,vmax=self.vmax)
                new_queue_item=Lidar_Queue_Item(new_line,t)
                self.lidar_state_queue.put(new_queue_item)
            else:
                new_line=mlab.plot3d(x,y,z,t,tube_radius=1,reset_zoom=False,colormap='Reds',vmin=self.vmin,vmax=self.vmax,opacity=0)
                new_queue_item=Lidar_Queue_Item(new_line,t)
                self.lidar_state_queue.put(new_queue_item)
            new_sphere=mlab.points3d(x1,y1,z1,reset_zoom=False,color=(1,1,1))
            ms_sphere=new_sphere.mlab_source
            self.sphere_state_queue.put(ms_sphere)
        else:
            if not(self.off):
                old_queue_item=self.lidar_state_queue.get()
                old_line=old_queue_item.lidar_line
                old_line.actor.property.opacity=1
                old_line.mlab_source.set(x=x,y=y,z=z,scalars=t,reset_zoom=False,vmin=self.vmin,vmax=self.vmax)
                old_queue_item.line=old_line
                old_queue_item.lidar_output=t
                self.lidar_state_queue.put(old_queue_item)
            else:
                old_queue_item=self.lidar_state_queue.get()
                old_line=old_queue_item.lidar_line
                old_line.actor.property.opacity=0
                old_queue_item.lidar_line=old_line
                old_queue_item.lidar_output=t
                self.lidar_state_queue.put(old_queue_item)
            old_sphere=self.sphere_state_queue.get()
            old_sphere.set(x=x1,y=y1,z=z1,reset_zoom=False,color=(1,1,1))
            self.sphere_state_queue.put(old_sphere)
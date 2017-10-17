# Miles E. Allen, 12 October 2017

import math
import queue
import states
import simulation
import lidar
import numpy as np
from mayavi import mlab

# A class to hold information regarding the Gondola.
class Gondola():
    cloud=None
    cloud_is_visible=None
    state=None
    state_queue=queue.Queue()
    current_position=(300,750,0)
    current_position_np=[300,750,0]
    gondola_azimuth=0
    gondola_elevation=0
    gondola_speed=3
    reference_direction=[0,1,0]
    view_distance=50
    iteration=0
    paused=False
    lidar=None
    command_queue=None
    command_latency=0
    lidar_off=False
    scan_angle=0
    
    az1_label=None
    az2_label=None
    position1_label=None
    position2_label=None
    
    lidar_azimuth=0
    lidar_elevation=0
    
    def __init__(self,position,wait,c_l,max_size):
        self.iteration=wait
        self.set_position(position)
        if (wait>0):
            self.state_queue=queue.Queue(maxsize=(wait+1))
            self.initial_stacking(wait)
        else:
            self.state_queue=queue.Queue()
        self.paused=False
        self.lidar=lidar.Lidar(maxsize=max_size)
        self.command_latency=c_l
        self.lidar.off=False
        self.scan_angle=45
        self.cloud_is_visible=True
        self.init_cloud()
#        self.init_planet()

    # This method creates the mesh of the cloud.
    def create_mesh(self,start,end,radius=50):
        deg=np.mgrid[0:360:((16+1)*1j)]
        rad=np.radians(deg)
        yy=(start,end)
        y,r=np.meshgrid(yy,rad)
        x=radius*np.sin(r)
        z=radius*np.cos(r)
        cloud_disk=mlab.mesh(x,y,z,color=(1,1,1),opacity=0.5)
        self.cloud.append(cloud_disk)

    def init_cloud(self):
        self.cloud=[]
        for i in range(0,50,2):
            self.create_mesh(start=(30*i),end=(30*(i+1)))

    def init_planet(self, zz=-40000, rr=504000, N=61):
        deg=np.mgrid[0:360:((N)*1j)]
        rad=np.radians(deg)
        phi,r=np.meshgrid(rad,(0,rr))
        x=r*np.sin(phi)
        y=r*np.cos(phi)
        z=0*phi + zz;
        mlab.mesh(x,y,z,color=(0,1,0))

    # Maybe this will work ...
    def initial_stacking(self,delay):
        for i in range(delay):
            self.move_gondola()
            state=states.Gondola_State(self.get_position(),self.get_azimuth(),self.get_elevation(),self.get_speed())
            self.state_queue.put(state)
    
    # Returns time since start of simulation.
    def return_time(self):
        return self.iteration
    
    def mode_select(self,mode):
        if (mode=="LIDAR Multi Scan"):
            self.command_queue.add(lambda: self.set_hv_seq())
            self.command_queue.add(lambda: self.turn_lidar_on())
        if (mode=="LIDAR Horizontal Scan"):
            self.command_queue.add(lambda: self.set_h_seq())
            self.command_queue.add(lambda: self.turn_lidar_on())
        if (mode=="LIDAR Vertical Scan"):
            self.command_queue.add(lambda: self.set_v_seq())
            self.command_queue.add(lambda: self.turn_lidar_on())
        if (mode=="LIDAR OFF"):
            self.command_queue.add(lambda: self.turn_lidar_off())
    
    def set_h_seq(self):
        self.lidar.seq=self.lidar.h_seq
    
    def set_v_seq(self):
        self.lidar.seq=self.lidar.v_seq
    
    def set_hv_seq(self):
        self.lidar.seq=self.lidar.hv_seq
    
    def turn_lidar_on(self):
        self.lidar_off=False
        self.lidar.off=False
    
    def turn_lidar_off(self):
        self.lidar_off=True
        self.lidar.off=True
    
    def turn_cloud_off(self):
        for disk in self.cloud:
            disk.actor.property.opacity=0
        self.cloud_is_visible=False
    
    def turn_cloud_on(self):
        for disk in self.cloud:
            disk.actor.property.opacity=1
        self.cloud_is_visible=True
    
    def trim_positions(self,position):
        x,y,z=position
        x=math.floor(x*10)/10
        y=math.floor(y*10)/10
        z=math.floor(z*10)/10
        return (x,y,z)
    
    # This function should update the camera and view with the next state in the queue.
    def advance_in_queue(self):
        current_state=self.state_queue.get()
        x,y,z=current_state.get_position()
        self.lidar.lidar_line(current_state.get_azimuth(),current_state.get_elevation(),current_state.get_position())
        mlab.view(distance=self.view_distance,focalpoint=current_state.get_position())
        
        # Update the labels in the GUI
        string_1="Last Commanded Azimuth: ("+str(self.get_azimuth())+")"
        string_2="Last Recorded Azimuth: ("+str(current_state.get_azimuth())+")"
        string_3="Last Commanded Position: "+str(self.trim_positions(self.get_position()))
        string_4="Last Recorded Position: "+str(self.trim_positions(current_state.get_position()))
        self.az1_label.setText(string_1)
        self.az2_label.setText(string_2)
        self.position1_label.setText(string_3)
        self.position2_label.setText(string_4)
    
    # This method is the basic function that mandates all other changes in the gondola class.
    # This function occurs once every second (on the clock timeout) unless paused.
    def default(self):
        if not self.paused:
            self.lidar.scan()
            print(" >>STATE",self.iteration,":: ",end='')
            if (self.command_latency != 0):
                if not(self.command_queue.is_empty()):
                    while not(self.command_queue.is_empty()):
                        c=self.command_queue.execute()
                        if ((self.return_time()-c[1])>=self.command_latency):
                            c[0]()
                        else:
                            self.command_queue.add_left(c)
                            break
            else:
                while not(self.command_queue.is_empty()):
                    c=self.command_queue.execute()[0]
                    c()
            self.move_gondola()
            
            view=mlab.view()
            if view!=None:
                self.view_distance=view[2]
#            print("view_distance:", self.view_distance)
            state=states.Gondola_State(self.get_position(),self.get_azimuth(),self.get_elevation(),self.get_speed())
            self.state_queue.put(state)
            print("GA:",self.get_azimuth(),"LA:",self.lidar.lidar_azimuth,"LE:",self.lidar.lidar_elevation)
            
            self.advance_in_queue()
            self.iteration+=1

    def move_gondola(self):
        x1,y1,z1=self.get_position()
        el_ma=simulation.elevation_matrix(self.get_elevation())
        az_ma=simulation.azimuth_matrix(self.get_azimuth())
        dot=np.matmul(el_ma,az_ma)
        x2,y2,z2=self.get_speed()*np.matmul(self.reference_direction,dot)
        # Assigning new, final positions.
        x=x1+x2
        y=y1+y2
        z=z1+z2
        self.set_position((x,y,z))
    
    def get_position(self):
        return self.current_position
    
    def set_position(self,position):
        x1,y1,z1=position
        x2,y2,z2=self.get_position()
        self.current_position=(x1,y1,z1)
        self.current_position_np=[x1,y1,z1]
    
    def get_elevation(self):
        return self.gondola_elevation
    
    def get_azimuth(self):
        return self.gondola_azimuth
    
    def set_azimuth(self, azimuth):
        self.gondola_azimuth=azimuth
    
    def get_speed(self):
        return self.gondola_speed

    def set_speed(self, speed):
        self.gondola_speed=speed

    def set_angle(self, angle):
        self.lidar.max_angle=angle

    def pan(self,direction):
        azimuth=self.get_azimuth()
        if (direction==-1):
            self.set_azimuth(azimuth-10)
        if (direction==1):
            self.set_azimuth(azimuth+10)
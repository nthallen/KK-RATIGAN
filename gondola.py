# Miles E. Allen, 12 October 2017

import queue
import states
import simulation
import lidar
import numpy as np
from mayavi import mlab

# A class to hold information regarding the Gondola.
class Gondola():
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
        self.lidar_off=False
        self.scan_angle=45
    
    # Maybe this will work ...
    def initial_stacking(self,delay):
        for i in range(delay):
            self.move_gondola()
            state=states.Gondola_State(self.get_position(),self.get_azimuth(),self.get_elevation(),self.get_speed())
            self.state_queue.put(state)
    
    def return_time(self):
        return self.iteration
    
    # This function should update the camera and view with the next state in the queue.
    def advance_in_queue(self):
        current_state=self.state_queue.get()
        x,y,z=current_state.get_position()
        self.lidar.lidar_line(current_state.get_azimuth(),current_state.get_elevation(),current_state.get_position(),self.lidar_off)
        mlab.view(distance=self.view_distance,focalpoint=current_state.get_position())
        
        # Update the labels in the GUI
        string_1="Last Commanded Azimuth: ("+str(self.get_azimuth())+")"
        string_2="Last Recorded Azimuth: ("+str(current_state.get_azimuth())+")"
        self.az1_label.setText(string_1)
        self.az2_label.setText(string_2)
    
    # This method is the basic function that mandates all other changes in the gondola class.
    # This function occurs once every second (on the clock timeout) unless paused.
    def default(self):
        if not self.paused:
            self.lidar.scan(self.scan_angle)
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
            
            # This line is just for test purposes
            #self.pan(-1)
            
            view=mlab.view()
            if view!=None:
                self.view_distance=view[2]
            state=states.Gondola_State(self.get_position(),self.get_azimuth(),self.get_elevation(),self.get_speed())
            self.state_queue.put(state)
            print(state.get_azimuth(),state.get_elevation(),state.get_position(),state.get_speed())
            
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

    def pan(self,direction):
        azimuth=self.get_azimuth()
        if (direction==-1):
            self.set_azimuth(azimuth-2)
        if (direction==1):
            self.set_azimuth(azimuth+2)
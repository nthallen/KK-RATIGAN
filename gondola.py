# Miles E. Allen, 12 October 2017

import math
import queue
import states
import simulation
import lidar
import numpy as np
import cloud
from mayavi import mlab

# A class to hold information regarding the Gondola.
class Gondola():
    rotation_severity=None
    new_cloud=None
    cloud_is_visible=None
    state=None
    state_queue=queue.Queue()
    current_position=(0,0,0)
    current_position_np=[0,0,0]
    
    # There are multiple frames of reference used in this model due to independent
    # use of subsystems. The first is the global coordinate system, which we define
    # here to be east in the x-direction and north in the y-direction, with the
    # origin as the starting point of the simulation.
    
    # The second is the gondola coordinate system. The z-axis is up and the y-axis
    # is the forward direction.
    # The origin of the coordinate system is currently defined to be the center
    # of the LIDAR elevation mirror.
    
    # The third refers to the LIDAR pointing direction. There may be others.
    
    # Angle in degrees from north clockwise. At 0 degrees azimuth, the gondola
    # reference direction is pointing north.
    gondola_azimuth=0
    # Rotation about the gondola's x-axis (pitch). Positive values within reason
    # mean pointing up.
    gondola_elevation=0
    # Rotation about the gondola's y-axis. Positive values within reason mean
    # tipping to the right.
    gondola_roll=0
    gondola_speed=3
    # Defines the forward direction in the gondola's coordinate system.
    # Note that this value is a constant.
    reference_direction=[0,1,0]
    # By convention, this is the origin of the gondola coordinate system. If this
    # point changes, the LIDAR angle calculations need to be modified.
    lidar_mirror_location=[0,0,0]
    # Defines the gondola's forward direction over the global coordinate system.
    direction_vector=None
    
    view_distance=50
    iteration=0
    paused=False
    lidar=None
    command_queue=None
    command_latency=0
    lidar_off=True
    cloud_spray_off=False
    scan_angle=0
    graph_on=None
    gondola_cloud_data=None
    current_circle=None

    # The location of the cloud-spewing nozzle, location gondola_length away from
    # the gondola's center in the gondola coordinate system.
    nozzle_position=None
    gondola_length=None
    
    az1_label=None
    az2_label=None
    position1_label=None
    position2_label=None
    
    lidar_azimuth=None
    lidar_elevation=None
    shear_azimuth=None
    shear_magnitude=None
    
    def __init__(self,position,wait,c_l,max_size,r_s,res,s_a,s_m):
        self.direction_vector=[0,1,0]
        self.gondola_length=1.5
        self.iteration=wait
        self.set_position(position)
        if (wait>0):
            self.state_queue=queue.Queue(maxsize=(wait+1))
            self.initial_stacking(wait)
        else:
            self.state_queue=queue.Queue()
        self.paused=False
        self.lidar=lidar.Lidar(maxsize=max_size,resolution=res)
        self.command_latency=c_l
        self.lidar.off=True
        self.scan_angle=45
        self.cloud_is_visible=True
        #self.init_planet(zz=-40000*.005, rr=504000*.005)
        self.graph_on=False
        self.lidar_azimuth=0
        self.nozzle_position=np.array(self.get_position())-(self.gondola_length/2)*np.array(self.direction_vector)
        self.rotation_severity=r_s
        self.shear_azimuth=s_a
        self.shear_magnitude=s_m
        self.init_cloud()

    # This method creates the mesh of the cloud.
    def create_mesh(self):
        if self.cloud_is_visible:
            self.new_cloud.draw_mesh()

    def init_cloud(self):
        self.new_cloud=cloud.Cloud(16,150,self.shear_magnitude,self.shear_azimuth,self)
        self.current_circle=self.new_cloud.current_circle

    def place_circle(self):
        time=self.iteration
        distance=self.calculate_distance()
        if (distance>=0):
            new_origin=self.nozzle_position+(10-distance)*np.array(self.direction_vector)
            self.new_cloud.add_circle(50,new_origin,time,self.direction_vector,self.new_cloud)

    def calculate_distance(self):
        current=self.current_circle
        distance_vector=np.array(self.get_position())-current.origin
        return np.dot(distance_vector,current.normal_vector)

    def init_planet(self,zz=-40000,rr=504000,N=61):
        deg=np.mgrid[0:360:((N)*1j)]
        rad=np.radians(deg)
        phi,r=np.meshgrid(rad,(0,rr))
        x=r*np.sin(phi)
        y=r*np.cos(phi)
        z=0*phi + zz;
        mlab.mesh(x,y,z,color=(0,0.3,0))

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
            self.lidar.lidar_azimuth=self.get_azimuth()
            self.lidar_azimuth=self.get_azimuth()
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
        self.cloud_is_visible=False
        self.new_cloud.visible=False
        xx=np.zeros(np.shape(self.new_cloud.P_x))
        yy=np.zeros(np.shape(self.new_cloud.P_y))
        zz=np.zeros(np.shape(self.new_cloud.P_z))
        s=np.zeros(np.shape(xx))
        self.new_cloud.ring.mlab_source.reset(x=xx,y=yy,z=zz,scalars=s)
        self.new_cloud.ring.mlab_source.set(x=xx,y=yy,z=zz)

    def turn_cloud_on(self):
        self.cloud_is_visible=True
        self.new_cloud.visible=True
        xx=self.new_cloud.P_x
        yy=self.new_cloud.P_y
        zz=self.new_cloud.P_z
        s=np.ones(np.shape(xx))
        self.new_cloud.ring.mlab_source.reset(x=xx,y=yy,z=zz,scalars=s)
        self.new_cloud.ring.mlab_source.set(x=xx,y=yy,z=zz,scalars=s)

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
        if (self.graph_on):
            self.lidar.graph_lidar_results()
    
    def update_camera(self):
        camera_view=mlab.view()
        camera_azimuth=camera_view[0]
        camera_elevation=camera_view[1]
        camera_distance=camera_view[2]
        camera_viewpoint=camera_view[3]
        
        x1=math.sin(math.radians(camera_elevation))*math.cos(math.radians(camera_azimuth))
        y1=math.sin(math.radians(camera_elevation))*math.sin(math.radians(camera_azimuth))
        z1=math.cos(math.radians(camera_elevation))

        print("XYZ: {0:.2f},{1:.2f},{2:.2f}".format(x1,y1,z1))
        
        x1*=camera_distance
        y1*=camera_distance
        z1*=camera_distance

        x2,y2,z2=camera_viewpoint

        x3=x2-x1
        y3=y2-y1
        z3=z2-z1

        new_focal_point=x3,y3,z3

        print(camera_viewpoint)
        print(new_focal_point)

        new_distance=camera_distance*2

        mlab.view(azimuth=camera_azimuth,elevation=camera_elevation,distance=new_distance,focalpoint=new_focal_point)

    # This method is the basic function that mandates all other changes in the gondola class.
    # This function occurs once every second (on the clock timeout) unless paused.
    def default(self):
        if not self.paused:
            if not (self.lidar_off):
                self.lidar.scan()
            print(" >> STATE",self.iteration,"   ::",self.trim_positions(self.get_position()))
            #print(" >>STATE",self.iteration,":: ",end='')
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
            # New code added as of 18 Jan 2018
            self.new_cloud.age()
            if not (self.cloud_spray_off):
                self.place_circle()
                self.current_circle=self.new_cloud.current_circle
            self.create_mesh()
            
            view=mlab.view()
            if view!=None:
                self.view_distance=view[2]
                #print("dist: {0:.2f}".format(self.view_distance))
            #print("view_distance:", self.view_distance)
            state=states.Gondola_State(self.get_position(),self.get_azimuth(),self.get_elevation(),self.get_speed())
            self.state_queue.put(state)
            #print("GA:",self.get_azimuth(),"LA:",self.lidar.lidar_azimuth,
            #      "LE: {0:.2f}".format(self.lidar.lidar_elevation),
            #      "View Az: {0:.2f}".format(view[0]),
            #      "El: {0:.2f}".format(view[1]),
            #      "dist: {0:.2f}".format(self.view_distance),
            #      "focus:", view[3])
            
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
        self.nozzle_position=np.array(self.get_position())-(self.gondola_length/2)*np.array(self.direction_vector)

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

    def turn_lidar(self,direction):
        azimuth=self.get_azimuth()
        if (direction==-1):
            self.set_azimuth(azimuth-10)
        if (direction==1):
            self.set_azimuth(azimuth+10)

    def turn_lidar_scan(self,direction):
        azimuth=self.lidar_azimuth
        if (direction==-1):
            self.lidar_azimuth=azimuth-10
            self.lidar.lidar_azimuth=azimuth-10
        if (direction==1):
            self.lidar_azimuth=azimuth+10
            self.lidar.lidar_azimuth=azimuth+10

    def turn(self,direction):
        azimuth=self.get_azimuth()
        if (direction==-1):
            self.set_azimuth(azimuth-self.rotation_severity)
        if (direction==1):
            self.set_azimuth(azimuth+self.rotation_severity)
        self.direction_vector=[np.sin(np.radians(self.get_azimuth())),np.cos(np.radians(self.get_azimuth())),0]
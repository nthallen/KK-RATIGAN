# Miles E. Allen, 12 October 2017

from pyface.qt import QtGui, QtCore

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
    
    output_file=None
    
    view_distance=50
    iteration=0
    paused=False
    lidar=None
    command_queue=None
    command_latency=0
    cloud_spray_off=False
    scan_angle=0
    graph_on=None
    gondola_cloud_data=None
    current_circle=None

    # The location of the cloud-spewing nozzle, location gondola_length away from
    # the gondola's center in the gondola coordinate system.
    nozzle_position=None
    gondola_length=None
    
    shear_azimuth=None
    shear_magnitude=None
    
    distance_to_next_circle=None
    stuff_quantum=None
    circle_distance=None
    
    popup_window=None
    popup_window_open=None
    
    lr_gondola_azimuth=None
    lc_gondola_azimuth=None
    lr_lidar_azimuth=None
    lc_lidar_azimuth=None
    lr_gondola_lidar_azimuth=None
    lc_gondola_lidar_azimuth=None
    lr_gondola_position=None
    lc_gondola_position=None
    
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
        self.nozzle_position=np.array(self.get_position())-(self.gondola_length/2)*np.array(self.direction_vector)
        self.rotation_severity=r_s
        self.shear_azimuth=s_a
        self.shear_magnitude=s_m
        self.init_cloud()
        self.lidar.new_cloud=self.new_cloud
        self.distance_to_next_circle=10
        self.stuff_quantum=200
        self.circle_distance=10
        self.output_file = open('gondola_output.txt', 'w')
        self.output_file.write(">GONDOLA OUTPUT\n")
        self.create_popup_horizontal()

    def create_popup_horizontal(self):
        self.popup_window=simulation.MyPopUp()
        self.popup_window_open=False
        self.popup_window.setWindowTitle("Gondola Flight Data")
        layout = QtGui.QGridLayout(self.popup_window)
        
        width=2
        
        text_font=QtGui.QFont("Courier New",12)
        text_font.setBold(True)
        number_font=QtGui.QFont("Courier New",12)
        
        label_labels_1=QtGui.QLabel()
        label_labels_1.setText("")
        label_labels_lc=QtGui.QLabel()
        label_labels_lc.setText("LC")
        label_labels_lc.setToolTip("Last Commanded")
        label_labels_lc.setFont(text_font)
        label_labels_lc.setAlignment(QtCore.Qt.AlignRight)
        label_labels_lr=QtGui.QLabel()
        label_labels_lr.setText("LR")
        label_labels_lr.setToolTip("Last Recorded")
        label_labels_lr.setFont(text_font)
        label_labels_lr.setAlignment(QtCore.Qt.AlignRight)
        
        label_lr_gondola_azimuth=QtGui.QLabel()
        label_lr_gondola_azimuth.setText("GA")
        label_lr_gondola_azimuth.setToolTip("Gondola Azimuth")
        label_lr_gondola_azimuth.setFont(text_font)
        label_lr_gondola_azimuth.setAlignment(QtCore.Qt.AlignCenter)
        self.lr_gondola_azimuth=QtGui.QLabel()
        self.lr_gondola_azimuth.setText("0")
        self.lr_gondola_azimuth.setFont(number_font)
        self.lr_gondola_azimuth.setStyleSheet("color: green")
        self.lr_gondola_azimuth.setFrameShape(QtGui.QFrame.Panel)
        self.lr_gondola_azimuth.setFrameShadow(QtGui.QFrame.Sunken)
        self.lr_gondola_azimuth.setLineWidth(width)
        lr_gondola_azimuth_units=QtGui.QLabel()
        lr_gondola_azimuth_units.setText("deg")
        lr_gondola_azimuth_units.setToolTip("Degrees")
        lr_gondola_azimuth_units.setFont(text_font)
        lr_gondola_azimuth_units.setAlignment(QtCore.Qt.AlignCenter)
        
        label_lc_gondola_azimuth=QtGui.QLabel()
        label_lc_gondola_azimuth.setText("LC Gondola Azimuth")
        self.lc_gondola_azimuth=QtGui.QLabel()
        self.lc_gondola_azimuth.setText("0")
        self.lc_gondola_azimuth.setFont(number_font)
        self.lc_gondola_azimuth.setStyleSheet("color: green")
        self.lc_gondola_azimuth.setFrameShape(QtGui.QFrame.Panel)
        self.lc_gondola_azimuth.setFrameShadow(QtGui.QFrame.Sunken)
        self.lc_gondola_azimuth.setLineWidth(width)
        lc_gondola_azimuth_units=QtGui.QLabel()
        lc_gondola_azimuth_units.setText("D")
        
        label_lr_lidar_azimuth=QtGui.QLabel()
        label_lr_lidar_azimuth.setText("LA")
        label_lr_lidar_azimuth.setToolTip("LIDAR Azimuth")
        label_lr_lidar_azimuth.setFont(text_font)
        label_lr_lidar_azimuth.setAlignment(QtCore.Qt.AlignCenter)
        self.lr_lidar_azimuth=QtGui.QLabel()
        self.lr_lidar_azimuth.setText("0")
        self.lr_lidar_azimuth.setFont(number_font)
        self.lr_lidar_azimuth.setStyleSheet("color: green")
        self.lr_lidar_azimuth.setFrameShape(QtGui.QFrame.Panel)
        self.lr_lidar_azimuth.setFrameShadow(QtGui.QFrame.Sunken)
        self.lr_lidar_azimuth.setLineWidth(width)
        lr_lidar_azimuth_units=QtGui.QLabel()
        lr_lidar_azimuth_units.setText("deg")
        lr_lidar_azimuth_units.setToolTip("Degrees")
        lr_lidar_azimuth_units.setFont(text_font)
        lr_lidar_azimuth_units.setAlignment(QtCore.Qt.AlignCenter)
        
        label_lc_lidar_azimuth=QtGui.QLabel()
        label_lc_lidar_azimuth.setText("LC LIDAR Azimuth")
        self.lc_lidar_azimuth=QtGui.QLabel()
        self.lc_lidar_azimuth.setText("0")
        self.lc_lidar_azimuth.setFont(number_font)
        self.lc_lidar_azimuth.setStyleSheet("color: green")
        self.lc_lidar_azimuth.setFrameShape(QtGui.QFrame.Panel)
        self.lc_lidar_azimuth.setFrameShadow(QtGui.QFrame.Sunken)
        self.lc_lidar_azimuth.setLineWidth(width)
        lc_lidar_azimuth_units=QtGui.QLabel()
        lc_lidar_azimuth_units.setText("D")
        
        label_lr_gondola_lidar_azimuth=QtGui.QLabel()
        label_lr_gondola_lidar_azimuth.setText("G+LA")
        label_lr_gondola_lidar_azimuth.setToolTip("Gondola + LIDAR Azimuth")
        label_lr_gondola_lidar_azimuth.setFont(text_font)
        label_lr_gondola_lidar_azimuth.setAlignment(QtCore.Qt.AlignCenter)
        self.lr_gondola_lidar_azimuth=QtGui.QLabel()
        self.lr_gondola_lidar_azimuth.setText("0")
        self.lr_gondola_lidar_azimuth.setFont(number_font)
        self.lr_gondola_lidar_azimuth.setStyleSheet("color: green")
        self.lr_gondola_lidar_azimuth.setFrameShape(QtGui.QFrame.Panel)
        self.lr_gondola_lidar_azimuth.setFrameShadow(QtGui.QFrame.Sunken)
        self.lr_gondola_lidar_azimuth.setLineWidth(width)
        lr_gondola_lidar_azimuth_units=QtGui.QLabel()
        lr_gondola_lidar_azimuth_units.setText("deg")
        lr_gondola_lidar_azimuth_units.setToolTip("Degrees")
        lr_gondola_lidar_azimuth_units.setFont(text_font)
        lr_gondola_lidar_azimuth_units.setAlignment(QtCore.Qt.AlignCenter)
        
        label_lc_gondola_lidar_azimuth=QtGui.QLabel()
        label_lc_gondola_lidar_azimuth.setText("LC Gondola + LIDAR Azimuth")
        self.lc_gondola_lidar_azimuth=QtGui.QLabel()
        self.lc_gondola_lidar_azimuth.setText("0")
        self.lc_gondola_lidar_azimuth.setFont(number_font)
        self.lc_gondola_lidar_azimuth.setStyleSheet("color: green")
        self.lc_gondola_lidar_azimuth.setFrameShape(QtGui.QFrame.Panel)
        self.lc_gondola_lidar_azimuth.setFrameShadow(QtGui.QFrame.Sunken)
        self.lc_gondola_lidar_azimuth.setLineWidth(width)
        lc_gondola_lidar_azimuth_units=QtGui.QLabel()
        lc_gondola_lidar_azimuth_units.setText("D")
        
        label_lr_gondola_position=QtGui.QLabel()
        label_lr_gondola_position.setText("Gondola Position")
        label_lr_gondola_position.setFont(text_font)
        label_lr_gondola_position.setAlignment(QtCore.Qt.AlignCenter)
        self.lr_gondola_position=QtGui.QLabel()
        self.lr_gondola_position.setText("(0,0,0)")
        self.lr_gondola_position.setFont(number_font)
        self.lr_gondola_position.setStyleSheet("color: green")
        self.lr_gondola_position.setFrameShape(QtGui.QFrame.Panel)
        self.lr_gondola_position.setFrameShadow(QtGui.QFrame.Sunken)
        self.lr_gondola_position.setLineWidth(width)
        lr_gondola_position_units=QtGui.QLabel()
        lr_gondola_position_units.setText("m")
        lr_gondola_position_units.setToolTip("Meters")
        lr_gondola_position_units.setFont(text_font)
        lr_gondola_position_units.setAlignment(QtCore.Qt.AlignCenter)
        
        label_lc_gondola_position=QtGui.QLabel()
        label_lc_gondola_position.setText("LC Gondola Position")
        self.lc_gondola_position=QtGui.QLabel()
        self.lc_gondola_position.setText("(0,0,0)")
        self.lc_gondola_position.setFont(number_font)
        self.lc_gondola_position.setStyleSheet("color: green")
        self.lc_gondola_position.setFrameShape(QtGui.QFrame.Panel)
        self.lc_gondola_position.setFrameShadow(QtGui.QFrame.Sunken)
        self.lc_gondola_position.setLineWidth(width)
        lc_gondola_position_units=QtGui.QLabel()
        lc_gondola_position_units.setText("M")
        
        layout.addWidget(label_labels_1,0,0)
        layout.addWidget(label_labels_lr,1,0)
        layout.addWidget(label_labels_lc,2,0)
        
        layout.addWidget(label_lr_gondola_azimuth,0,1)
        layout.addWidget(self.lr_gondola_azimuth,1,1)
        layout.addWidget(self.lc_gondola_azimuth,2,1)
        layout.addWidget(lr_gondola_azimuth_units,3,1)

        layout.addWidget(label_lr_lidar_azimuth,0,2)
        layout.addWidget(self.lr_lidar_azimuth,1,2)
        layout.addWidget(self.lc_lidar_azimuth,2,2)
        layout.addWidget(lr_lidar_azimuth_units,3,2)
        
        layout.addWidget(label_lr_gondola_lidar_azimuth,0,3)
        layout.addWidget(self.lr_gondola_lidar_azimuth,1,3)
        layout.addWidget(self.lc_gondola_lidar_azimuth,2,3)
        layout.addWidget(lr_gondola_lidar_azimuth_units,3,3)
        
        layout.addWidget(label_lr_gondola_position,0,4)
        layout.addWidget(self.lr_gondola_position,1,4)
        layout.addWidget(self.lc_gondola_position,2,4)
        layout.addWidget(lr_gondola_position_units,3,4)
        
        #######################################################################
        self.popup_window.show()

    def create_popup(self):
        # The goal is to create three columns; Label, number, and corresponding unit
        self.popup_window=simulation.MyPopUp()
        self.popup_window_open=False
        self.popup_window.setWindowTitle("Gondola Flight Data")
        layout = QtGui.QGridLayout(self.popup_window)
        
        label_lr_gondola_azimuth=QtGui.QLabel()
        label_lr_gondola_azimuth.setText("LR Gondola Azimuth")
        self.lr_gondola_azimuth=QtGui.QLabel()
        self.lr_gondola_azimuth.setText("0")
        lr_gondola_azimuth_units=QtGui.QLabel()
        lr_gondola_azimuth_units.setText("Degrees")
        
        label_lc_gondola_azimuth=QtGui.QLabel()
        label_lc_gondola_azimuth.setText("LC Gondola Azimuth")
        self.lc_gondola_azimuth=QtGui.QLabel()
        self.lc_gondola_azimuth.setText("0")
        lc_gondola_azimuth_units=QtGui.QLabel()
        lc_gondola_azimuth_units.setText("Degrees")
        
        label_lr_lidar_azimuth=QtGui.QLabel()
        label_lr_lidar_azimuth.setText("LR LIDAR Azimuth")
        self.lr_lidar_azimuth=QtGui.QLabel()
        self.lr_lidar_azimuth.setText("0")
        lr_lidar_azimuth_units=QtGui.QLabel()
        lr_lidar_azimuth_units.setText("Degrees")
        
        label_lc_lidar_azimuth=QtGui.QLabel()
        label_lc_lidar_azimuth.setText("LC LIDAR Azimuth")
        self.lc_lidar_azimuth=QtGui.QLabel()
        self.lc_lidar_azimuth.setText("0")
        lc_lidar_azimuth_units=QtGui.QLabel()
        lc_lidar_azimuth_units.setText("Degrees")
        
        label_lr_gondola_lidar_azimuth=QtGui.QLabel()
        label_lr_gondola_lidar_azimuth.setText("LR Gondola + LIDAR Azimuth")
        self.lr_gondola_lidar_azimuth=QtGui.QLabel()
        self.lr_gondola_lidar_azimuth.setText("0")
        lr_gondola_lidar_azimuth_units=QtGui.QLabel()
        lr_gondola_lidar_azimuth_units.setText("Degrees")
        
        label_lc_gondola_lidar_azimuth=QtGui.QLabel()
        label_lc_gondola_lidar_azimuth.setText("LC Gondola + LIDAR Azimuth")
        self.lc_gondola_lidar_azimuth=QtGui.QLabel()
        self.lc_gondola_lidar_azimuth.setText("0")
        lc_gondola_lidar_azimuth_units=QtGui.QLabel()
        lc_gondola_lidar_azimuth_units.setText("Degrees")
        
        label_lr_gondola_position=QtGui.QLabel()
        label_lr_gondola_position.setText("LR Gondola Position")
        self.lr_gondola_position=QtGui.QLabel()
        self.lr_gondola_position.setText("(0,0,0)")
        lr_gondola_position_units=QtGui.QLabel()
        lr_gondola_position_units.setText("Meters")
        
        label_lc_gondola_position=QtGui.QLabel()
        label_lc_gondola_position.setText("LC Gondola Position")
        self.lc_gondola_position=QtGui.QLabel()
        self.lc_gondola_position.setText("(0,0,0)")
        lc_gondola_position_units=QtGui.QLabel()
        lc_gondola_position_units.setText("Meters")
        
        layout.addWidget(label_lc_gondola_azimuth,0,0)
        layout.addWidget(self.lc_gondola_azimuth,0,1)
        layout.addWidget(lc_gondola_azimuth_units,0,2)
        
        layout.addWidget(label_lr_gondola_azimuth,1,0)
        layout.addWidget(self.lr_gondola_azimuth,1,1)
        layout.addWidget(lr_gondola_azimuth_units,1,2)
        
        layout.addWidget(label_lc_lidar_azimuth,2,0)
        layout.addWidget(self.lc_lidar_azimuth,2,1)
        layout.addWidget(lc_lidar_azimuth_units,2,2)
        
        layout.addWidget(label_lr_lidar_azimuth,3,0)
        layout.addWidget(self.lr_lidar_azimuth,3,1)
        layout.addWidget(lr_lidar_azimuth_units,3,2)
        
        layout.addWidget(label_lc_gondola_lidar_azimuth,4,0)
        layout.addWidget(self.lc_gondola_lidar_azimuth,4,1)
        layout.addWidget(lc_gondola_lidar_azimuth_units,4,2)
        
        layout.addWidget(label_lr_gondola_lidar_azimuth,5,0)
        layout.addWidget(self.lr_gondola_lidar_azimuth,5,1)
        layout.addWidget(lr_gondola_lidar_azimuth_units,5,2)
        
        layout.addWidget(label_lc_gondola_position,6,0)
        layout.addWidget(self.lc_gondola_position,6,1)
        layout.addWidget(lc_gondola_position_units,6,2)
        
        layout.addWidget(label_lr_gondola_position,7,0)
        layout.addWidget(self.lr_gondola_position,7,1)
        layout.addWidget(lr_gondola_position_units,7,2)
        
        #######################################################################
        self.popup_window.show()
    
    # This method updates all of the necessary values in the display window.
    def update_popup_window(self,current_state):
        self.lc_gondola_azimuth.setText(str('{:4d}'.format(self.get_azimuth())))
        self.lr_gondola_azimuth.setText(str('{:4d}'.format(current_state.get_azimuth())))
        self.lc_lidar_azimuth.setText(str('{:4d}'.format(self.lidar.lidar_azimuth-self.get_azimuth())))
        self.lr_lidar_azimuth.setText(str('{:4d}'.format(current_state.get_lidar_azimuth()-current_state.get_azimuth())))
        self.lc_gondola_lidar_azimuth.setText(str('{:4d}'.format(self.lidar.lidar_azimuth)))
        self.lr_gondola_lidar_azimuth.setText(str('{:4d}'.format(current_state.get_lidar_azimuth())))
        
        x1,y1,z1=self.get_position()
        x2,y2,z2=current_state.get_position()
        
        x1_float=str('{:07.1f}'.format(x1).lstrip('0'))
        y1_float=str('{:07.1f}'.format(y1).lstrip('0'))
        z1_float=str('{:07.1f}'.format(z1).lstrip('0'))
        
        x2_float=str('{:07.1f}'.format(x2).lstrip('0'))
        y2_float=str('{:07.1f}'.format(y2).lstrip('0'))
        z2_float=str('{:07.1f}'.format(z2).lstrip('0'))
        
        self.lc_gondola_position.setText(str("("+'{:>7}, {:>7}, {:>7}'.format(x1_float,y1_float,z1_float))+")")
        self.lr_gondola_position.setText(str("("+'{:>7}, {:>7}, {:>7}'.format(x2_float,y2_float,z2_float))+")")

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
            new_origin=self.nozzle_position+(self.circle_distance-distance)*np.array(self.direction_vector)
            self.new_cloud.add_circle(50,new_origin,time,self.direction_vector,self.new_cloud)
            
            n=len(self.new_cloud.circles)
            cell_prev=self.new_cloud.circles[n-3]
            cell_current=self.new_cloud.circles[n-2]
            
            d1=distance
            d2=self.distance_to_next_circle
            cell_prev.stuff+=self.stuff_quantum*(d2/(d1+d2))
            cell_current.stuff+=self.stuff_quantum*(d1/(d1+d2))
            
            self.distance_to_next_circle=self.circle_distance-distance
        else:
            # We're still in the old cell
            n=len(self.new_cloud.circles)
            cell_current=self.new_cloud.circles[n-2]
            cell_current.stuff+=self.stuff_quantum
            
            self.distance_to_next_circle=-distance

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
            state=states.Gondola_State(self.get_position(),self.get_azimuth(),self.get_elevation(),self.get_speed(),self.lidar.lidar_azimuth)
            self.state_queue.put(state)

    # Returns time since start of simulation.
    def return_time(self):
        return self.iteration

    def mode_select(self,mode):
        if (mode=="LIDAR Multi Scan"):
            self.command_queue.add(lambda: self.set_hv_seq())
            self.command_queue.add(lambda: self.turn_lidar_on())
        if (mode=="LIDAR Horizontal Scan"):
            self.command_queue.add(lambda: self.set_lidar_azimuth_to_gondola_azimuth())
            self.command_queue.add(lambda: self.set_h_seq())
            self.command_queue.add(lambda: self.turn_lidar_on())
        if (mode=="LIDAR Vertical Scan"):
            self.command_queue.add(lambda: self.set_lidar_azimuth_to_gondola_azimuth())
            self.command_queue.add(lambda: self.set_v_seq())
            self.command_queue.add(lambda: self.turn_lidar_on())
        if (mode=="LIDAR OFF"):
            self.command_queue.add(lambda: self.set_lidar_azimuth_to_gondola_azimuth())
            self.command_queue.add(lambda: self.turn_lidar_off())

    def set_lidar_azimuth_to_gondola_azimuth(self):
        self.lidar.lidar_azimuth=self.get_azimuth()
        self.lidar.lidar_elevation=self.get_elevation()

    def set_h_seq(self):
        self.lidar.seq=self.lidar.h_seq

    def set_v_seq(self):
        self.lidar.seq=self.lidar.v_seq

    def set_hv_seq(self):
        self.lidar.seq=self.lidar.hv_seq

    def turn_lidar_on(self):
        self.lidar.off=False

    def turn_lidar_off(self):
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

    def draw_lidar_line(self,current_state):
        self.lidar.lidar_line(current_state.get_lidar_azimuth(),current_state.get_elevation(),current_state.get_position())

    # This function should update the camera and view with the next state in the queue.
    def advance_in_queue(self):
        current_state=self.state_queue.get()
        x,y,z=current_state.get_position()
        self.draw_lidar_line(current_state)
        #self.lidar.lidar_line(current_state.get_lidar_azimuth(),current_state.get_elevation(),current_state.get_position())
        mlab.view(distance=self.view_distance,focalpoint=current_state.get_position())
        
        # Update the labels in the GUI
        # "LC" means "last commanded" and "LR" means "last recorded"
        # as in, LC is the most up to date information, and LR is what most
        # recently took place on-screen
        
        self.update_popup_window(current_state)
        
        # Update the graph, if applicable
        if (self.graph_on):
            self.lidar.graph_lidar_results()
    
    # This method writes information on the cloud's "stuff" to the output file.
    def print_relevant_information(self):
        print_string = " >>STATE "+str(self.iteration)+"::\n"
        self.output_file.write(print_string)
        for i in range(len(self.new_cloud.circles)):
            print_string_cloud = "   >>>circle["+str(i)+"].stuff="+str(self.new_cloud.circles[i].stuff)+"\n"
            self.output_file.write(print_string_cloud)
    
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
            
            if not(self.lidar.off):
                self.lidar.scan()
            #print(" >> STATE",self.iteration,"   ::",self.trim_positions(self.get_position()))
            #print(" >>STATE",self.iteration,":: ",end='')
            
            # New code added as of 18 Jan 2018
            self.new_cloud.age()
            if not (self.cloud_spray_off):
                self.place_circle()
                self.current_circle=self.new_cloud.current_circle
            self.create_mesh()
            self.print_relevant_information()
            
            view=mlab.view()
            if view!=None:
                self.view_distance=view[2]
                #print("dist: {0:.2f}".format(self.view_distance))
            #print("view_distance:", self.view_distance)
            state=states.Gondola_State(self.get_position(),self.get_azimuth(),self.get_elevation(),self.get_speed(),self.lidar.lidar_azimuth)
            self.state_queue.put(state)
            #print("GA:",self.get_azimuth(),"LA:",self.lidar.lidar_azimuth,
            #      "LE: {0:.2f}".format(self.lidar.lidar_elevation),
            #      "View Az: {0:.2f}".format(view[0]),
            #      "El: {0:.2f}".format(view[1]),
            #      "dist: {0:.2f}".format(self.view_distance),
            #      "focus:", view[3])
            
            self.advance_in_queue()
            self.iteration+=1

    # This method moves the gondola's position, according to the direction vector and the speed.
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

    def turn_lidar_scan(self,direction):
        azimuth=self.lidar.lidar_azimuth
        if (direction==-1):
            self.lidar.lidar_azimuth=azimuth-self.rotation_severity
        if (direction==1):
            self.lidar.lidar_azimuth=azimuth+self.rotation_severity

    def turn(self,direction):
        azimuth=self.get_azimuth()
        lidar_azimuth=self.lidar.lidar_azimuth
        if (direction==-1):
            self.set_azimuth(azimuth-self.rotation_severity)
            self.lidar.lidar_azimuth=lidar_azimuth-self.rotation_severity
        if (direction==1):
            self.set_azimuth(azimuth+self.rotation_severity)
            self.lidar.lidar_azimuth=lidar_azimuth+self.rotation_severity
        self.direction_vector=[np.sin(np.radians(self.get_azimuth())),np.cos(np.radians(self.get_azimuth())),0]
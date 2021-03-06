# Miles E. Allen, 12 October 2017
# FLIGHT SCHEDULED FOR MAY 2018
# 5m, 15m, 30m, 2.4m bin size, higher collection rate than 1Hz

# First, and before importing any Enthought packages, set the ETS_TOOLKIT
# environment variable to qt4, to tell Traits that we will use Qt.
import os
os.environ['ETS_TOOLKIT'] = 'qt4'
# By default, the PySide binding will be used. If you want the PyQt bindings
# to be used, you need to set the QT_API environment variable to 'pyqt'
#os.environ['QT_API'] = 'pyqt'

# To be able to use PySide or PyQt4 and not run in conflicts with traits,
# we need to import QtGui and QtCore from pyface.qt
from pyface.qt import QtGui, QtCore
# Alternatively, you can bypass this line, but you need to make sure that
# the following lines are executed before the import of PyQT:
#   import sip
#   sip.setapi('QString', 2)
import math
import vtk
import interactions
import gondola

from mayavi import mlab
from traits.api import HasTraits, Instance, on_trait_change
from traitsui.api import View, Item
from mayavi.core.ui.api import MayaviScene, MlabSceneModel, \
        SceneEditor

# This method returns a matrix based on angle.
def azimuth_matrix(angle):
    azimuth=math.radians(angle)
    caz=math.cos(azimuth)
    saz=math.sin(azimuth)
    return [[caz,-saz,0],[saz,caz,0],[0,0,1]]

# This method returns a matrix based on angle.
def elevation_matrix(angle):
    elevation=math.radians(angle)
    cel=math.cos(elevation)
    sel=math.sin(elevation)
    return [[1,0,0],[0,cel,sel],[0,-sel,cel]]

# This method sets the camera's initial view.
def setup_view():
    mlab.view(azimuth=-90,elevation=80,distance=20,focalpoint=(300,750,0))

################################################################################
#The actual visualization
class Visualization(HasTraits):
    scene = Instance(MlabSceneModel, ())

    @on_trait_change('scene.activated')
    def update_plot(self):
        # This function is called when the view is opened. We don't
        # populate the scene when the view is not yet open, as some
        # VTK features require a GLContext.
        
        # Nothing works, nothing ever works, what the fuck
        #fig=mlab.gcf()
        #fig=mlab.figure(bgcolor=(0.52,0.8,0.92))
        #mlab.figure(bgcolor=(0.52,0.8,0.92))
        self.scene.background = (0.42,0.7,0.82)
        self.scene.foreground = (0,0.2,0)
        # render backdrop
        # turns out if you try to render AN ENTIRE FUCKING PLANET
        # the cloud and dots don't show up because the resolution is off
        #earth=mlab.points3d(0,0,-6030000,mode='sphere',scale_factor=6000000,color=(0,0.25,0))
        #earth_horizon = mlab.barchart(0,0,-30000,mode='2dcircle', lateral_scale=200,scale_factor=30000, color=(0,0.25,0),reset_zoom=False)
        
        # For some reason this is the line that causes that annoying error message:
        #mlab.xlabel("origin")
        
        #s = np.random.random((1000, 1000))
        #earth=mlab.imshow(s,colormap='gist_earth')

        # Create Arrow Vectors
        a=[-300,0]
        b=[0,0]
        c=[0,0]
        mlab.quiver3d(a,b,c,color=(0.5,0,0),line_width=2.0,reset_zoom=False,scale_factor=1)
        x=[0,0]
        y=[0,300]
        z=[0,0]
        mlab.quiver3d(x,y,z,color=(0.5,0,0),line_width=2.0,reset_zoom=False,scale_factor=1)
        #,extent=[0,0,0,300,-300,-300]
        #,extent=[-300,0,0,0,-300,-300]

        # We can do normal mlab calls on the embedded scene.
        #self.scene.mlab.test_points3d()
        
    # the layout of the dialog screated
    view = View(Item('scene', editor=SceneEditor(scene_class=MayaviScene),
                     height=250, width=300, show_label=False),
                resizable=True # We need this to resize with the parent widget
                )

################################################################################
# The QWidget containing the visualization, this is pure PyQt4 code.
class MayaviQWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        layout = QtGui.QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        self.visualization = Visualization()

        # If you want to debug, beware that you need to remove the Qt
        # input hook.
        #QtCore.pyqtRemoveInputHook()
        #import pdb ; pdb.set_trace()
        #QtCore.pyqtRestoreInputHook()

        # The edit_traits call will generate the widget to embed.
        self.ui = self.visualization.edit_traits(parent=self,kind='subpanel').control
        
        layout.addWidget(self.ui)
        self.ui.setParent(self)

class MyPopUp(QtGui.QWidget):
    def __init__(self):
        QtGui.QWidget.__init__(self)

# Main method
if __name__ == "__main__":
    # THE IMPORTANT VARIABLES
    latency=0           # The number of iterations for gondola results to reach ground (downlink latency)
    command_latency=0   # The number of iterations for ground commands to reach the gondola (uplink latency)
    reliability=1       # The probability that a command will follow through to the gondola, 0 being 0% and 1 being 100%
    maxsize=36          # The maximum size of the graphical objects queues, both positional spheres and LIDAR lines
    rotation=10         # The number of degrees each turn command will move the direction of the gondola
    resolution=20       # The resolution of the LIDAR
    shear_azimuth=0     # Accounting for wind
    shear_magnitude=0   # In meters/second/kilometer

    vtk.vtkObject.GlobalWarningDisplayOff()
    #renderer=vtk.vtkRenderer()
    #renderer.SetBackground
    
    # Don't create a new QApplication, it would unhook the Events
    # set by Traits on the existing QApplication. Simply use the
    # '.instance()' method to retrieve the existing one.
    app = QtGui.QApplication.instance()
    container = QtGui.QWidget()
    container.setWindowTitle("Gondola Flight Simulation")
    # define a "complex" layout to test the behaviour
    layout = QtGui.QGridLayout(container)
    # put some stuff around mayavi
    label_list = []
    # for i in range(3):
        # for j in range(3):
            # if (i==1) and (j==1):continue
            # label = QtGui.QLabel(container)
            # label.setText("Your QWidget at (%d, %d)" % (i,j))
            # label.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
            # layout.addWidget(label, i, j)
            # label_list.append(label)
    mayavi_widget = MayaviQWidget(container)
    
    new_gondola = gondola.Gondola((0,0,0),wait=latency,c_l=command_latency,max_size=maxsize,r_s=rotation,res=resolution,s_a=shear_azimuth,s_m=shear_magnitude)
    new_gondola.lidar.gondola=new_gondola
    #new_gondola.new_cloud.gondola=new_gondola
    command_queue=interactions.Command_Queue(new_gondola,rel=reliability)
    new_gondola.command_queue=command_queue
    
    layout_2 = QtGui.QGridLayout()
    layout_display = QtGui.QGridLayout()
    layout_display_azimuth = QtGui.QGridLayout()
    layout_4 = QtGui.QGridLayout()
    layout_5 = QtGui.QGridLayout()
    layout.addWidget(mayavi_widget,0,0)

    speed_slider = interactions.SpeedSlider(new_gondola,command_queue)
    speed_slider.connect_value_changed()
    angle_slider = interactions.AngleSlider(new_gondola,command_queue)
    angle_slider.connect_value_changed()
    
    button_l = interactions.DirectionButton("Turn Left",-1,new_gondola,command_queue)
    button_r = interactions.DirectionButton("Turn Right",1,new_gondola,command_queue)
    button_l.connect_released()
    button_r.connect_released()
    
    lidar_button_l=interactions.LidarScanDirectionButton("LIDAR Scan Left",-1,new_gondola,command_queue)
    lidar_button_r=interactions.LidarScanDirectionButton("LIDAR Scan Right",1,new_gondola,command_queue)
    lidar_button_l.connect_released()
    lidar_button_r.connect_released()
    
    layout_2.addWidget(button_l,0,0)
    layout_2.addWidget(lidar_button_l,0,1)
    layout_2.addWidget(lidar_button_r,0,2)
    layout_2.addWidget(button_r,0,3)
    
    timer=QtCore.QTimer()
    timer.timeout.connect(new_gondola.default)
    timer.start(1000)
    
    pause_button=interactions.PauseButton("Pause",new_gondola)
    pause_button.connect_released()
    cloud_button=interactions.CloudButton("Cloud OFF",new_gondola)
    cloud_button.connect_released()
    cloud_spray_button=interactions.CloudSprayOff("Terminate Cloud Spray",new_gondola)
    cloud_spray_button.connect_released()
    
    combo_box = QtGui.QComboBox()
    combo_box.addItem("LIDAR OFF")
    combo_box.addItem("LIDAR Multi Scan")
    combo_box.addItem("LIDAR Horizontal Scan")
    combo_box.addItem("LIDAR Vertical Scan")
    combo_box.activated[str].connect(new_gondola.mode_select)
    
    layout_speed_slider=QtGui.QGridLayout()
    layout_speed_labels=QtGui.QGridLayout()
    layout_angle_slider=QtGui.QGridLayout()
    layout_angle_labels=QtGui.QGridLayout()
    
    speed_slider_name=QtGui.QLabel()
    speed_slider_name.setText("Gondola Movement Speed")
    speed_slider_name.setAlignment(QtCore.Qt.AlignCenter)
    speed_slider_min=QtGui.QLabel()
    speed_slider_min.setText("0")
    speed_slider_max=QtGui.QLabel()
    speed_slider_max.setText("3")
    speed_slider_max.setAlignment(QtCore.Qt.AlignRight)
    angle_slider_name=QtGui.QLabel()
    angle_slider_name.setText("LIDAR Scan Angle")
    angle_slider_name.setAlignment(QtCore.Qt.AlignCenter)
    angle_slider_min=QtGui.QLabel()
    angle_slider_min.setText("0")
    angle_slider_max=QtGui.QLabel()
    angle_slider_max.setText("45")
    angle_slider_max.setAlignment(QtCore.Qt.AlignRight)
    
    layout_speed_labels.addWidget(speed_slider_min,0,0)
    layout_speed_labels.addWidget(speed_slider_name,0,1)
    layout_speed_labels.addWidget(speed_slider_max,0,2)
    
    layout_angle_labels.addWidget(angle_slider_min,0,0)
    layout_angle_labels.addWidget(angle_slider_name,0,1)
    layout_angle_labels.addWidget(angle_slider_max,0,2)
    
    layout_speed_slider.addLayout(layout_speed_labels,0,0)
    layout_speed_slider.addWidget(speed_slider,1,0)
    
    layout_angle_slider.addLayout(layout_angle_labels,0,0)
    layout_angle_slider.addWidget(angle_slider,1,0)
    
    az1_display=QtGui.QLabel()
    az2_display=QtGui.QLabel()
    display_speed=QtGui.QLabel()
    display_angle=QtGui.QLabel()
    #display_speed.setStyleSheet("background-color: white")
    display_speed.setStyleSheet("color: green")
    #display_angle.setStyleSheet("background-color: white")
    display_angle.setStyleSheet("color: green")
    position1_display=QtGui.QLabel()
    position2_display=QtGui.QLabel()
    
    az1_display.setText("0")
    az2_display.setText("0")
    
    myFont=QtGui.QFont()
    myFont.setBold(True)
    display_speed.setFont(myFont)
    display_angle.setFont(myFont)
    
    display_speed.setText("(3)")
    display_angle.setText("(23)")
    #print(display_speed.width())
    #print(display_angle.width())
    #display_speed.resize(1000,display_speed.height())
    #display_angle.resize(1000,display_angle.height())
    #print(display_speed.width())
    #print(display_angle.width())
    #display_speed.setGeometry(QtCore.QRect(display_speed.x(),display_speed.y(),display_speed.width()+500,display_speed.height()))
    #display_angle.setGeometry(QtCore.QRect(display_angle.x(),display_angle.y(),display_angle.width()+500,display_angle.height()))
    position1_display.setText("0")
    position2_display.setText("0")
    
    speed_slider.speed_label=display_speed
    angle_slider.angle_label=display_angle
    
    layout_4.addWidget(display_speed,0,0)
    layout_4.addLayout(layout_speed_slider,0,1)
    layout_4.addWidget(display_angle,0,2)
    layout_4.addLayout(layout_angle_slider,0,3)
    
    layout_5.addWidget(pause_button,0,0)
    layout_5.addWidget(cloud_button,0,1)
    layout_5.addWidget(cloud_spray_button,0,2)
    
    #view_button=interactions.ViewButton("Distance", gondola)
    #view_button.connect_released()
    
    layout.addLayout(layout_2,1,0)
    layout.addLayout(layout_4,2,0)
    layout.addLayout(layout_5,3,0)
    layout.addWidget(combo_box,4,0)
    
    graph_button=interactions.LidarGraphButton("Graph LIDAR Data",new_gondola)
    graph_button.connect_released()
    layout.addWidget(graph_button,5,0)
    
    #layout.addWidget(view_button,6,0)
    
    container.show()
    window = QtGui.QMainWindow()
    window.setCentralWidget(container)
    window.show()
    setup_view()
    # Start the main event loop.
    app.exec_()
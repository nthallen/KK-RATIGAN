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
import numpy as np
import math
import queue
import copy

from mayavi import mlab
from traits.api import HasTraits, Instance, on_trait_change
from traitsui.api import View, Item
from mayavi.core.ui.api import MayaviScene, MlabSceneModel, \
        SceneEditor

RESOLUTION=100

# A class to hold information regarding the Gondola.
class Gondola():
    state_queue=queue.Queue()
    current_position=(300,750,0)
    current_position_np=[300,750,0]
    gondola_azimuth=0
    gondola_elevation=0
    gondola_speed=3
    reference_direction=[0,1,0]
    
    lidar_azimuth=0
    lidar_elevation=0
    
    def __init__(self,position,wait=0):
        self.set_position(position)
        self.state_queue=queue.Queue(maxsize=wait)
    
    # Hold gondola states in queue, only update when the timer expires?
    # With each popping of state off the queue, we update the camera, draw the lines and the points
    
    # This function should call the next function in the queue.
    def advance_in_queue(self):
        current_state=self.state_queue.get()
        x,y,z=current_state.get_position()
        mlab.points3d([x],[y],[z],reset_zoom=False,color=(1,1,1))
        lidar_line(current_state.get_azimuth(),current_state.get_elevation(),current_state.get_position())
        mlab.view(focalpoint=current_state.get_position())
    
    def default(self):
        self.move_gondola()
        current_state=copy.deepcopy(self)
        self.state_queue.put(current_state)
        self.advance_in_queue()
    
    # This function moves the gondola, indicating its current position with a
    # red star, and previous positions with white stars.
    def move_gondola(self):
        x1,y1,z1=self.get_position()
        el_ma=elevation_matrix(self.get_elevation())
        az_ma=azimuth_matrix(self.get_azimuth())
        dot=np.matmul(el_ma,az_ma)
        x2,y2,z2=self.get_speed()*np.matmul(self.reference_direction,dot)
        # Assigning new, final positions.
        x=x1+x2
        y=y1+y2
        z=z1+z2
        self.set_position((x,y,z))
        # Drawing the old and new points and the line from the LIDAR.
        #mlab.points3d([x1],[y1],[z1],reset_zoom=False,color=(1,1,1))
        #mlab.points3d([x],[y],[z],reset_zoom=False,color=(1,0,0))
        #lidar_line(self.get_azimuth(),self.get_elevation(),self.get_position())
    
    def get_position(self):
        return self.current_position
    
    def set_position(self,position):
        #view=mlab.view()
        #mlab_distance=50
        #if view!=None:
            #mlab_distance=view[2]
        x1,y1,z1=position
        x2,y2,z2=self.get_position()
        self.current_position=(x1,y1,z1)
        self.current_position_np=[x1,y1,z1]
        #if not(x1==x2 and y1==y2 and z1==z2):
            #mlab.view(distance=mlab_distance,focalpoint=self.get_position())
    
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
            self.set_azimuth(azimuth-5)
        if (direction==1):
            self.set_azimuth(azimuth+5)

# This method determines whether the given point is inside the cloud or not.
def is_in_cloud(position):
    x,y,z=position
    distance = math.sqrt((x-0)**2 + (z-0)**2)
    if distance <= 50:
        if (0 <= y <= 1500):
            return 1
        else:
            return 0
    return 0

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

# This method draws a line where the LIDAR instrument is pointing.
def lidar_line(azimuth,elevation,position):
    x1,y1,z1=position
    az_ma=azimuth_matrix(azimuth)
    el_ma=elevation_matrix(elevation)
    dot=np.matmul(el_ma,az_ma)
    x2,y2,z2=np.matmul([0,1,0],dot)
    bin_dist=np.mgrid[100:2000:(RESOLUTION*1j)]
    x = x1 + bin_dist*x2
    y = y1 + bin_dist*y2
    z = z1 + bin_dist*z2
    t = 0*bin_dist
    for i in range(len(bin_dist)):
        t[i] = is_in_cloud((x[i],y[i],z[i]))
    mlab.plot3d(x,y,z,t,tube_radius=1,reset_zoom=False,colormap='Greys')

# This method sets the camera's initial view.
def setup_view():
    mlab.view(azimuth=0,elevation=0,distance=50,focalpoint=(750,750,0))

# This method creates the mesh of the cloud.
def create_mesh(radius=50):
    deg=np.mgrid[0:360:((16+1)*1j)]
    rad=np.radians(deg)
    yy=(0,1500)
    y,r=np.meshgrid(yy,rad)
    x=radius*np.sin(r)
    z=radius*np.cos(r)
    mlab.mesh(x,y,z,color=(1,1,1),opacity=0.5)

################################################################################
#The actual visualization
class Visualization(HasTraits):
    scene = Instance(MlabSceneModel, ())

    @on_trait_change('scene.activated')
    def update_plot(self):
        # This function is called when the view is opened. We don't
        # populate the scene when the view is not yet open, as some
        # VTK features require a GLContext.

        create_mesh()
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

# A class for the gondola's directional buttons.
class DirectionButton(QtGui.QPushButton):
    label = "none"
    direction=0
    gondola=None
    
    def __init__(self,string,direction,gondola):
        QtGui.QPushButton.__init__(self,string)
        self.label=string
        self.direction=direction
        self.gondola=gondola
        
    def connect_released(self):
        self.released.connect(self.handle_released)
        
    def handle_released(self):
        gondola.pan(self.direction)

# A class for the slider that changes the speed of the gondola.
class SpeedSlider(QtGui.QSlider):
    value=3
    gondola=None
    def __init__(self,gondola):
        QtGui.QSlider.__init__(self, QtCore.Qt.Horizontal)
        QtGui.QSlider.setMinimum(self,0)
        QtGui.QSlider.setMaximum(self,3)
        QtGui.QSlider.setValue(self,3)
        self.gondola=gondola

    def connect_value_changed(self):
        self.valueChanged.connect(self.value_changed)
        
    def value_changed(self,value):
        self.value=value
        gondola.set_speed(value)

if __name__ == "__main__":
    setup_view()
    gondola = Gondola((750,750,0))
    # Don't create a new QApplication, it would unhook the Events
    # set by Traits on the existing QApplication. Simply use the
    # '.instance()' method to retrieve the existing one.
    app = QtGui.QApplication.instance()
    container = QtGui.QWidget()
    container.setWindowTitle("Embedding Mayavi in a PyQt4 Application")
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
    layout_2 = QtGui.QGridLayout()
    layout.addWidget(mayavi_widget,0,0)

    speed_slider = SpeedSlider(gondola)
    speed_slider.connect_value_changed()
    
    button_l = DirectionButton("Left",-1,gondola)
    button_r = DirectionButton("Right",1,gondola)
    button_l.connect_released()
    button_r.connect_released()
    
    layout_2.addWidget(button_l,0,0)
    layout_2.addWidget(button_r,0,1)
    
    timer=QtCore.QTimer()
    timer.timeout.connect(gondola.default)
    timer.start(1000)
    
    layout.addLayout(layout_2,1,0)
    layout.addWidget(speed_slider,2,0)
    container.show()
    window = QtGui.QMainWindow()
    window.setCentralWidget(container)
    window.show()

    # Start the main event loop.
    app.exec_()
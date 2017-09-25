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

from mayavi import mlab
from traits.api import HasTraits, Instance, on_trait_change
from traitsui.api import View, Item
from mayavi.core.ui.api import MayaviScene, MlabSceneModel, \
        SceneEditor

RESOLUTION=100

# A class to hold information regarding the Gondola.
class Gondola():
    current_position=(300,750,0)
    gondola_azimuth=0
    gondola_elevation=90
    gondola_speed=3
    
    lidar_azimuth=0
    lidar_elevation=90
    
    def __init__(self, position):
        pass
    
    def get_azimuth(self):
        return self.gondola_azimuth
    
    def set_azimuth(self, azimuth):
        self.gondola_azimuth = azimuth
    
    def get_speed(self):
        return self.gondola_speed

    def set_speed(self, speed):
        self.gondola_speed = speed

# Slider with thrust from 0m/s to 3m/s

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

# This method draws a line where the LIDAR instrument is pointing.
def lidar_line(azimuth,elevation,position):
    x1,y1,z1=position
    az = math.radians(azimuth)
    el = math.radians(elevation)
    caz = math.cos(az)
    saz = math.sin(az)
    cel = math.cos(el)
    sel = math.sin(el)
    azimuth_matrix=[[caz,-saz,0],[saz,caz,0],[0,0,1]]
    elevation_matrix=[[1,0,0],[0,cel,sel],[0,-sel,cel]]
    dot=np.matmul(elevation_matrix,azimuth_matrix)
    x2,y2,z2=np.matmul([0,1,0],dot)
    bin_dist=np.mgrid[100:2000:(RESOLUTION*1j)]
    x = x1 + bin_dist*x2
    y = y1 + bin_dist*y2
    z = z1 + bin_dist*z2
    t = 0*bin_dist
    for i in range(len(bin_dist)):
        t[i] = is_in_cloud((x[i],y[i],z[i]))
    mlab.plot3d(x,y,z,t,tube_radius=1,reset_zoom=False,colormap='Greys')

# This method automatically moves the gondola 3 meters forward upon timeout.
def default_movement():
    #azimuth,elevation,distance,focalpoint=mlab.view()
    #lidar_line(azimuth,elevation,focalpoint)
    mlab.move(forward=Gondola.get_speed(Gondola))

# This method pans the camera by 5 degrees.
def pan(direction):
    if (direction==-1):
        mlab.yaw(5)
    if (direction==1):
        mlab.yaw(-5)

# This method sets the camera's initial view.
def setup_view():
    mlab.view(azimuth=0,elevation=90,distance=50,focalpoint=(300,750,0))

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

class DirectionButton(QtGui.QPushButton):
    label = "none"
    direction=0
    def __init__(self, string="Button",direction=0):
        QtGui.QPushButton.__init__(self, string)
        self.label = string
        self.direction=direction
        
    def connect_released(self):
        self.released.connect(self.handle_released)
        
    def handle_released(self):
        #print(self.label, "Released", self.direction)
        pan(self.direction)

# A class for the slider that changes the speed of the gondola
class SpeedSlider(QtGui.QSlider):
    value=3
    def __init__(self):
        QtGui.QSlider.__init__(self, QtCore.Qt.Horizontal)
        QtGui.QSlider.setMinimum(self,0)
        QtGui.QSlider.setMaximum(self,3)
        QtGui.QSlider.setValue(self,3)

    def connect_value_changed(self):
        self.valueChanged.connect(self.value_changed)
        
    def value_changed(self,value):
        self.value=value
        Gondola.set_speed(Gondola,value)

if __name__ == "__main__":
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

    speed_slider = SpeedSlider()
    speed_slider.connect_value_changed()

    button_l = DirectionButton("Left",-1)
    button_r = DirectionButton("Right",1)
    button_l.connect_released()
    button_r.connect_released()
    
    layout_2.addWidget(button_l,0,0)
    layout_2.addWidget(button_r,0,1)
    setup_view()
    
    timer=QtCore.QTimer()    
    timer.timeout.connect(default_movement)
    timer.start(1000)
    
    layout.addLayout(layout_2,1,0)
    layout.addWidget(speed_slider,2,0)
    container.show()
    window = QtGui.QMainWindow()
    window.setCentralWidget(container)
    window.show()

    # Start the main event loop.
    app.exec_()
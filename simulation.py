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
import numpy as np
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
    mlab.view(azimuth=0,elevation=0,distance=0,focalpoint=(300,750,0))

# This method creates the mesh of the cloud.
def create_mesh(start,end,radius=50):
    deg=np.mgrid[0:360:((16+1)*1j)]
    rad=np.radians(deg)
    yy=(start,end)
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
        
        #fig=mlab.gcf()
        #fig=mlab.figure(bgcolor=(0.52,0.8,0.92))

        # render backdrop
        # turns out if you try to render AN ENTIRE FUCKING PLANET
        # the cloud and dots don't show up because the resolution is off
#        u=np.linspace(0,2*np.pi,10)
#        v=np.linspace(0,np.pi,10)
#        x=6371000*np.outer(np.cos(u),np.sin(v))
#        y=6371000*np.outer(np.sin(u),np.sin(v))
#        z=6371000*np.outer(np.ones(np.size(u)),np.cos(v))-6401000
#        mlab.mesh(x,y,z,color=(0, 0.5, 0))

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
        
        for i in range(0,50,2):
            create_mesh(start=(30*i),end=(30*(i+1)))
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

# Main method
if __name__ == "__main__":
    # The important stuff
    latency=0           # The number of iterations for results to reach ground
    command_latency=0   # The number of iterations for commands to reach the gondola
    reliability=1       # The % chance that a command will follow through
    maxsize=128         # The maximum size of the graphical objects queues

    vtk.vtkObject.GlobalWarningDisplayOff()
    #renderer=vtk.vtkRenderer()
    #renderer.SetBackground

    setup_view()
    gondola = gondola.Gondola((750,750,0),wait=latency,c_l=command_latency,max_size=maxsize)
    command_queue=interactions.Command_Queue(gondola,rel=reliability)
    gondola.command_queue=command_queue
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
    layout_3 = QtGui.QGridLayout()
    layout.addWidget(mayavi_widget,0,0)

    speed_slider = interactions.SpeedSlider(gondola,command_queue)
    speed_slider.connect_value_changed()
    
    button_l = interactions.DirectionButton("Pan Left",-1,gondola,command_queue)
    button_r = interactions.DirectionButton("Pan Right",1,gondola,command_queue)
    button_l.connect_released()
    button_r.connect_released()
    
    layout_2.addWidget(button_l,0,0)
    layout_2.addWidget(button_r,0,1)
    
    timer=QtCore.QTimer()
    timer.timeout.connect(gondola.default)
    timer.start(1000)
    
    pause_button=interactions.PauseButton("Pause/Play",gondola)
    pause_button.connect_released()
    off_button=interactions.OffButton("Lidar On/Off",gondola)
    off_button.connect_released()
    
    az1_display=QtGui.QLabel()
    az2_display=QtGui.QLabel()
    az1_display.setText("0")
    az2_display.setText("0")
    
    layout_3.addWidget(az1_display,0,0)
    layout_3.addWidget(az2_display,0,1)
    gondola.az1_label=az1_display
    gondola.az2_label=az2_display
    
    layout.addLayout(layout_2,1,0)
    layout.addWidget(speed_slider,2,0)
    layout.addWidget(pause_button,3,0)
    layout.addWidget(off_button,4,0)
    layout.addLayout(layout_3,5,0)
    
    container.show()
    window = QtGui.QMainWindow()
    window.setCentralWidget(container)
    window.show()

    # Start the main event loop.
    app.exec_()
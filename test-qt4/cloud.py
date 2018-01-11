# Miles E. Allen, 6 December 2017

import numpy as np
from mayavi import mlab
import os
import interactions
import vtk
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

from traits.api import HasTraits, Instance, on_trait_change
from traitsui.api import View, Item
from mayavi.core.ui.api import MayaviScene, MlabSceneModel, \
        SceneEditor

time=0

###############################################################################
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

        # Create Arrow Vectors
        a=[-3,0]
        b=[0,0]
        c=[0,0]
        mlab.quiver3d(a,b,c,color=(0.5,0,0),line_width=2.0,reset_zoom=False,scale_factor=1)
        x=[0,0]
        y=[0,3]
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

###############################################################################
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

###############################################################################

class gondola():
    queue_size=None
    queue=None
    x_pos=None
    y_pos=None
    z_pos=None
    current_circle=None
    cloud=None
    direction=None
    nozzle_position=None
    gondola_length=None

    def __init__(self,queue_size):
        self.gondola_length=1.5
        self.queue_size=queue_size
        self.queue=interactions.IndexableQueue(maxsize=self.queue_size)
        self.x_pos=0
        self.y_pos=0
        self.z_pos=0
        self.cloud=Cloud(16,150,1,(0,1,0))
        self.time=0
        self.direction=(0,1,0)
        self.nozzle_position=np.array(self.get_position())-(self.gondola_length/2)*np.array(self.direction)

    def get_position(self):
        self.y_pos+=3
        #print(self.x_pos,self.y_pos,self.z_pos)
        return self.x_pos,self.y_pos,self.z_pos

    def default_movement(self):
        global time
        time+=1
        self.cloud.age()
        self.current_circle=self.cloud.current_circle
        self.nozzle_position=np.array(self.get_position())-(self.gondola_length/2)*np.array(self.direction)
        x1,y1,z1=self.get_position()
        self.place_circle()
        if (self.queue.qsize()<self.queue_size):
            new_sphere=mlab.points3d(x1,y1,z1,reset_zoom=False,color=(1,1,1))
            self.queue.put(new_sphere.mlab_source)
        else:
            old_sphere=self.queue.get()
            old_sphere.set(x=x1,y=y1,z=z1,reset_zoom=False,color=(1,1,1))
            self.queue.put(old_sphere)
    
    def place_circle(self):
        global time
        direction=self.direction
        distance=self.calculate_distance()
        if (distance>=0):
            new_origin=self.nozzle_position+(10-distance)*np.array(direction)
            self.cloud.add_circle(25,new_origin,time,direction)
    
    def calculate_distance(self):
        current=self.current_circle
        distance_vector=np.array(self.nozzle_position)-current.origin
        return np.dot(distance_vector,current.normal_vector)

# A class to hold data on an individual circle for the cloud object.
class Circle():
    radius=None
    origin=None
    birth=None
    normal_vector=None
    
    def __init__(self,radius,origin,age,normal_vector):
        self.radius=radius
        self.origin=origin
        self.birth=age
        self.normal_vector=normal_vector
    
    def return_age(self):
        global time
        return time-self.birth

# A class to hold data on the new cloud object.
class Cloud():
    gondola=None
    circles=None
    age=None
    M=None
    N=None
    P=None
    P_x=None
    P_y=None
    P_z=None
    current_circle=None
    shear_magnitude=None
    shear_direction=None
    ring=None
    
    # N is the number of circles, M is the number of points around the circle.
    def __init__(self,M,N,s_m,s_d):
        self.M=M
        self.N=N
        self.circles=[]
        circle_1=Circle(25,(0,-5,0),0,(0,1,0))
        circle_2=Circle(25,(0,5,0),0,(0,1,0))
        self.circles.append(circle_1)
        self.circles.append(circle_2)
        self.current_circle=circle_2
        self.P_x=[]
        self.P_y=[]
        self.P_z=[]
        self.shear_magnitude=s_m
        self.shear_direction=s_d
        self.do_the_math()
        first_ring=mlab.mesh(self.P_x,self.P_y,self.P_z,color=(1,1,1),opacity=1,reset_zoom=False)
        self.ring=first_ring.mlab_source
    
    def age(self):
        scale_factor=1
        for circle in self.circles:
            circle.radius+=scale_factor
    
    def draw_mesh(self):
        self.do_the_math()
        #s=np.ones_like(self.P_x)
        self.ring.reset(x=self.P_x,y=self.P_y,z=self.P_z,scalars=s)
        
        #radius + scale_factor*age
        
        #,color=(1,1,1),opacity=1,reset_zoom=False
        #self.ring=mlab.mesh(self.P_x,self.P_y,self.P_z,color=(1,1,1),opacity=1,reset_zoom=False)
        
        #for old_ring in self.rings:
        #    old_ring.actor.property.opacity=0
        #new_ring=mlab.mesh(self.P_x,self.P_y,self.P_z,color=(1,1,1),opacity=1,reset_zoom=False)
        #self.rings.append(new_ring)
    
    def add_circle(self,radius,origin,age,normal_vector):
        new_circle=Circle(radius,origin,age,normal_vector)
        self.circles.append(new_circle)
        self.current_circle=new_circle
        #print("New circle at",new_circle.origin,"created at time",new_circle.birth)
        self.draw_mesh()
    
    def do_the_math(self):
        N=len(self.circles)
        origins=np.zeros((3,N))
        for i in range(N):
            for j in range(3):
                origins[j][i]=self.circles[i].origin[j]
        m_col=np.ones((self.M+1,1))
        
        O_x=m_col*origins[0]
        O_y=m_col*origins[1]
        O_z=m_col*origins[2]
        
        self.P=np.empty(shape=[N,self.M],dtype=tuple)
        self.P_x=np.empty(shape=[N,self.M],dtype=tuple)
        self.P_y=np.empty(shape=[N,self.M],dtype=tuple)
        self.P_z=np.empty(shape=[N,self.M],dtype=tuple)
        
        # restructure everything so that np.cosg and np.sing is calculated only once
        # the other stuff updates with every circle added
        
        rad_v=((np.array(range(self.M+1))*2*np.pi)/self.M)
        cos_v=np.cos(rad_v)
        sin_v=np.sin(rad_v)
        ncos_grid,cos_grid=np.meshgrid(np.array(range(N)),np.array(cos_v))
        nsin_grid,sin_grid=np.meshgrid(np.array(range(N)),np.array(sin_v))
        
        X=np.zeros((3,N))
        Z=np.zeros((3,N))
        R=np.zeros((1,N))
        
        for n in range(N):
            Y=self.circles[n].normal_vector
            z_n=np.array([0,0,1])
            x_n=np.cross(Y,z_n)
            R[0][n]=self.circles[n].radius
            
            for i in range(3):
                X[i][n]=x_n[i]
                Z[i][n]=z_n[i]
        
        self.P_x=O_x+np.multiply(m_col*np.multiply(X[0],R),cos_grid)+np.multiply(m_col*np.multiply(Z[0],R),sin_grid)
        self.P_y=O_y+np.multiply(m_col*np.multiply(X[1],R),cos_grid)+np.multiply(m_col*np.multiply(Z[1],R),sin_grid)
        self.P_z=O_z+np.multiply(m_col*np.multiply(X[2],R),cos_grid)+np.multiply(m_col*np.multiply(Z[2],R),sin_grid)

# Main method
if __name__ == "__main__":
    vtk.vtkObject.GlobalWarningDisplayOff()
    queue_size=10
    
    app = QtGui.QApplication.instance()
    container = QtGui.QWidget()
    container.setWindowTitle("Gondola Flight Simulation")
    # define a "complex" layout to test the behaviour
    layout = QtGui.QGridLayout(container)
    # put some stuff around mayavi
    label_list = []
    mayavi_widget = MayaviQWidget(container)
    layout.addWidget(mayavi_widget,0,0)
    
    new_gondola=gondola(queue_size)
    new_gondola.cloud.gondola=new_gondola
    
    timer=QtCore.QTimer()
    timer.timeout.connect(new_gondola.default_movement)
    timer.start(1000)
    
    container.show()
    window = QtGui.QMainWindow()
    window.setCentralWidget(container)
    window.show()
    # Start the main event loop.
    app.exec_()
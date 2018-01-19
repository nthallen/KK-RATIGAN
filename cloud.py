# Miles E. Allen, 18 January 2018

import numpy as np
from mayavi import mlab
import os
os.environ['ETS_TOOLKIT'] = 'qt4'

# A class to hold data on an individual circle for the cloud object.
class Circle():
    radius=None
    origin=None
    birth=None
    normal_vector=None
    cloud=None
    
    def __init__(self,radius,origin,age,normal_vector,cl):
        self.radius=radius
        self.origin=origin
        self.birth=age
        self.normal_vector=normal_vector
        self.cloud=cl
    
    def return_age(self):
        time=self.cloud.gondola.return_time()
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
    visible=None
    
    # N is the number of circles, M is the number of points around the circle.
    def __init__(self,M,N,s_m,s_d):
        self.M=M
        self.N=N
        self.circles=[]
        circle_1=Circle(50,(0,-5,0),0,(0,1,0),self)
        circle_2=Circle(50,(0,5,0),0,(0,1,0),self)
        self.circles.append(circle_1)
        self.circles.append(circle_2)
        self.current_circle=circle_2
        self.P_x=[]
        self.P_y=[]
        self.P_z=[]
        self.shear_magnitude=s_m
        self.shear_direction=s_d
        self.do_the_math()
        self.ring=mlab.mesh(self.P_x,self.P_y,self.P_z,color=(1,1,1),opacity=1,reset_zoom=False)
        self.visible=True
    
    def age(self):
        scale_factor=0.1
        for circle in self.circles:
            circle.radius+=scale_factor
    
    def draw_mesh(self):
        self.do_the_math()
        s=np.ones(np.shape(self.P_x))
        if self.visible:
            self.ring.mlab_source.reset(x=self.P_x,y=self.P_y,z=self.P_z,scalars=s)
            self.ring.mlab_source.set(x=self.P_x,y=self.P_y,z=self.P_z)
    
    def add_circle(self,radius,origin,age,normal_vector,cloud):
        new_circle=Circle(radius,origin,age,normal_vector,cloud)
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
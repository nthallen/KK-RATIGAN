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
    stuff=None
    
    def __init__(self,radius,origin,age,normal_vector,cl):
        self.radius=radius
        self.origin=origin
        self.birth=age
        self.normal_vector=normal_vector
        self.cloud=cl
        self.stuff=0
    
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
    shear_azimuth=None
    shear_direction=None
    ring=None
    visible=None
    
    # N is the number of circles, M is the number of points around the circle.
    def __init__(self,M,N,s_m,s_a,g):
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
        self.shear_azimuth=s_a
        self.gondola=g
        self.shear_direction=[np.sin(np.radians(self.shear_azimuth)),np.cos(np.radians(self.shear_azimuth)),0]
        self.do_the_math()
        self.ring=mlab.mesh(self.P_x,self.P_y,self.P_z,color=(1,1,1),opacity=1,reset_zoom=False)
        self.visible=True
        self.distance_to_next_circle=10
    
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
    
    # This method returns an array of all circles' ages currently in the cloud.
    def create_age_array(self):
        ages=[]
        for circle in self.circles:
            ages.append(circle.return_age())
        return ages

    def d(self,P,n):
        #ages=self.create_age_array()
        circle_c=self.circles[n]
        P_1=P-((P[2]-circle_c.origin[2])*self.shear_magnitude*circle_c.return_age()*np.array(self.shear_direction))/1000
        dN=np.dot((P_1-circle_c.origin),circle_c.normal_vector)
        if (dN>=0):
            #P_2=(P_1-dN*circle_c.normal_vector)
            P_2=(P_1-circle_c.origin)-dN*np.array(circle_c.normal_vector)
            dR=np.sqrt(np.sum(np.square(P_2)))
        else:
            dR=0
        return (dN,dR)
    
    # Given point P, this function determines which cell - if any - P is in
    # and calculates the number density of the appropriate cell.
    def check_cell(self,P):
        for n in range(len(self.circles)):
            dN,dR=self.d(P,n)
            dN_1,dR_1=self.d(P,(n+1))
            if (dN >= 0) and (dN_1 < 0):
                # point P is in the cell between circles n and n+1
                cell_current=self.circles[n]
                c_r=cell_current.radius
                density_1=(cell_current.stuff/(c_r*np.sqrt(2*np.pi)))*np.exp(-(1/2)*np.power((dR/c_r),2))
                return density_1
            else:
                return 0
    
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
        
        circle_ages=np.zeros((1,N))
        
        for n in range(N):
            Y=self.circles[n].normal_vector
            z_n=np.array([0,0,1])
            x_n=np.cross(Y,z_n)
            R[0][n]=self.circles[n].radius
            circle_ages[0][n]=self.circles[n].return_age()
            
            for i in range(3):
                X[i][n]=x_n[i]
                Z[i][n]=z_n[i]
        
        age_grid=m_col*circle_ages
        
        self.P_z=O_z+np.multiply(m_col*np.multiply(X[2],R),cos_grid)+np.multiply(m_col*np.multiply(Z[2],R),sin_grid)
        z_age_grid=np.multiply(age_grid,self.P_z)/1000
        self.P_x=O_x+np.multiply(m_col*np.multiply(X[0],R),cos_grid)+np.multiply(m_col*np.multiply(Z[0],R),sin_grid)+(z_age_grid*self.shear_magnitude*self.shear_direction[0])
        self.P_y=O_y+np.multiply(m_col*np.multiply(X[1],R),cos_grid)+np.multiply(m_col*np.multiply(Z[1],R),sin_grid)+(z_age_grid*self.shear_magnitude*self.shear_direction[1])
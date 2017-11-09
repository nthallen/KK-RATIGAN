import numpy as np
from mayavi import mlab
import sys
import math

def init_clouds():
    operator=(1/2)
    operator_2=(math.sqrt(2)/2)
    
    pre_x=np.mgrid[-500:500:11j]
    x_1,y_1=np.meshgrid(pre_x,pre_x)
    z_1=0*x_1
    
    x_2=operator*(x_1-y_1)
    y_2=operator*(-x_1+y_1)
    z_2=operator_2*(x_1+y_1)
    
    cloud_1=mlab.mesh(x_1,y_1,z_1,color=(0,0,0),representation='wireframe')
    cloud_2=mlab.mesh(x_2,y_2,z_2,color=(0,0,0),representation='wireframe')

def init_planet(zz=-40000, rr=504000, N=61):
    deg=np.mgrid[0:360:((N)*1j)]
    rad=np.radians(deg)
    phi,r=np.meshgrid(rad,(0,rr))
    x=r*np.sin(phi)
    y=r*np.cos(phi)
    z=0*phi + zz;
    planet=mlab.mesh(x,y,z,color=(0,0.2,0))
    planet.actor.property.opacity=0

def main():
    if (str(sys.argv[1])=="on"):
        init_planet()
    init_clouds()
    mlab.view(-30,75,793,(500,-500,0))
    #mlab.view(-30,75,1500,(0,0,0))
    mlab.show()
main()
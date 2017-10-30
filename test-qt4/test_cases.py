import numpy as np
from mayavi import mlab
import sys

def init_cloud():
    pre_x=np.mgrid[0:1000:11j]
    x,y=np.meshgrid(pre_x,pre_x)
    z=0*x
    mlab.mesh(x,y,z,color=(1,1,1),representation='wireframe')

def init_planet(zz=-40000, rr=504000, N=61):
    deg=np.mgrid[0:360:((N)*1j)]
    rad=np.radians(deg)
    phi,r=np.meshgrid(rad,(0,rr))
    x=r*np.sin(phi)
    y=r*np.cos(phi)
    z=0*phi + zz;
    mlab.mesh(x,y,z,color=(0,0.2,0))
    #planet.actor.property.opacity=0

def main():
    if (str(sys.argv[1])=="on"):
        init_planet()
    init_cloud()
    mlab.view(-45,80,2000,(0,750,0))
    mlab.show()
main()
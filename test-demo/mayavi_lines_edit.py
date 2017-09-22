import math
import numpy as np
from mayavi import mlab

# This method displays cloud lines with adjustments to account for wind movement.
def display_cloud(divisions,a=0,b=0,c=0):
    a=360/divisions
    rad=math.radians(a)
    accumulating_rad=rad
    for i in range(divisions):
        x=50*math.cos(accumulating_rad)
        z=50*math.sin(accumulating_rad)+50
        accumulating_rad+=rad
        line=mlab.plot3d([x+a,x+a],[0+b,1500+b],[z+c,z+c],tube_radius=0.5,color=(1,1,1))

# The method creates the mesh of the cloud.
def create_mesh(radius=50):
    deg = np.mgrid[0:360:((16+1)*1j)];
    rad = np.radians(deg)
    yy = (0,1500)
    y,r = np.meshgrid(yy,rad)
    x = radius*np.sin(r)
    z = radius*np.cos(r)
    print(r)
    print(x)
    print(y)
    print(z)
    mlab.mesh(x,y,z, color=(1,1,1), opacity=0.2)

#    deg=np.mgrid[0:360:16j]
#    rad=list(map(math.radians,deg))
#    x_coord=list(map(math.sin,rad))*50
#    y_coord=(0,1500)
#    z_coord=list(map(math.cos,rad))*50
#    x,z=np.meshgrid(x_coord,z_coord)
#    mlab.mesh(x,y,z)

# Main method.
def main():
    #options.backend='envisage'
    fig=mlab.figure(bgcolor=(0.52,0.8,1))
    #radius_line=mlab.plot3d([0,0],[0,1500],[50,50],tube_radius=50,color=(1,1,1))

    #radius_extent=(-50,50,0,1500,0,100)
    #mlab.outline(extent=radius_extent,figure=fig)
        
    #display_cloud(16)
    
    #xlabel("x axis")
    #ylabel("y axis")
    #zlabel("z axis")
    
    #mlab.axes(radius_line,extent=radius_extent,ranges=radius_extent,xlabel="x axis",ylabel="y axis",zlabel="z axis")
    #mlab.show()
    create_mesh()
    mlab.show()
main()

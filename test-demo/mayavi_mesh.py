import numpy as np
from mayavi import mlab

deg = np.mgrid[0:360:((16+1)*1j)];
rad = np.radians(deg)
radius = 50
yy = (1,1500)
y,r = np.meshgrid(yy,rad)
x = radius * np.sin(r)
z = radius * np.cos(r)
fig = mlab.figure(bgcolor=(.7,.95,1))
s = mlab.mesh(x, y, z, color=(1,1,1), opacity=0.2)
mlab.show()

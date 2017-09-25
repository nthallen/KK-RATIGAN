import numpy as np
from time import sleep
from mayavi import mlab
x, y = np.mgrid[0:3:1,0:3:1]
s = mlab.surf(x, y, np.asarray(x*0.1, 'd'))
mlab.draw()

for i in range(10):
    s.mlab_source.scalars = np.asarray(x*0.1*(i+1), 'd')
    sleep(1)
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 11 16:28:09 2018

@author: nort
"""

import numpy as np
from numpy import arange, pi, cos, sin

from traits.api import HasTraits, Range, Instance, \
        on_trait_change
from traitsui.api import View, Item, Group

from mayavi.core.api import PipelineBase
from mayavi.core.ui.api import MayaviScene, SceneEditor, \
                MlabSceneModel


dphi = pi/1000.
phi = arange(0.0, 2*pi + 0.5*dphi, dphi, 'd')

def curve(n_mer, n_long):
    mu = phi*n_mer
    x = cos(mu) * (1 + cos(n_long * mu/n_mer)*0.5)
    y = sin(mu) * (1 + cos(n_long * mu/n_mer)*0.5)
    z = 0.5 * sin(n_long*mu/n_mer)
    t = sin(mu)
    return x, y, z, t


class MyModel(HasTraits):
    n_meridional    = Range(1, 30, 6, )#mode='spinner')
    n_longitudinal  = Range(5, 25, 10, )#mode='spinner')

    scene = Instance(MlabSceneModel, ())

    plot = Instance(PipelineBase)
    
    rtype,resolution = n_longitudinal.get_default_value()
    xyrange = np.mgrid[-resolution:resolution+1:1]
    xgrid,ygrid = np.meshgrid(xyrange,xyrange)
    rgrid = np.sqrt(np.multiply(xgrid,xgrid) + np.multiply(ygrid,ygrid))


    # When the scene is activated, or when the parameters are changed, we
    # update the plot.
    @on_trait_change('n_meridional,n_longitudinal,scene.activated')
    def update_plot(self):
        # x, y, z, t = curve(self.n_meridional, self.n_longitudinal)
        res_changed = 0
        if self.n_longitudinal != self.resolution:
            self.resolution = self.n_longitudinal;
            self.xyrange = np.mgrid[-self.resolution:self.resolution+1:1];
            self.xgrid,self.ygrid = np.meshgrid(self.xyrange,self.xyrange)
            self.rgrid = np.sqrt(np.multiply(self.xgrid,self.xgrid) +
                                 np.multiply(self.ygrid,self.ygrid))
            res_changed = 1
            
        zgrid = 1.5*np.cos(self.rgrid*pi/self.n_meridional)
        if self.plot is None:
            self.plot = self.scene.mlab.mesh(self.xgrid, self.ygrid, zgrid,
                                             scalars=np.ones(np.shape(zgrid)),
                                             color=(0.9,0.9,0.9))
        elif res_changed != 0:
            self.plot.mlab_source.reset(x=self.xgrid, y=self.ygrid, z=zgrid,
                                             scalars=np.ones(np.shape(zgrid)))
        else:
            self.plot.mlab_source.set(z=zgrid)


    # The layout of the dialog created
    view = View(Item('scene', editor=SceneEditor(scene_class=MayaviScene),
                     height=250, width=300, show_label=False),
                Group(
                        '_', 'n_meridional', 'n_longitudinal',
                     ),
                resizable=True,
                )

my_model = MyModel()
my_model.configure_traits()
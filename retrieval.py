# Miles E. Allen, 3 November 2017

import numpy as np

def lidar_retrieval():
    bin_length=20
    bins=np.mgrid[100:2000:bin_length]
    beta=1e6
    cloud_range=600
    cloud_radius=50
    cloud_max_density=10
    background=1
    density_1=cloud_max_density*np.exp(-(1/2)*np.power(((bins-cloud_range)/cloud_radius),2))
    density=density_1+background
    signal=beta*bin_length*density/np.power(bins,2)
    print(signal)
    SN=0*signal
    for i in range (1,len(signal)):
        SN[i]=np.random.poisson(signal[i])
    NSN=SN*np.power(bins,2)/(beta*bin_length)
    print(NSN)
lidar_retrieval()
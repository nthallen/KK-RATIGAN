# This file tests some wonkers code.
# I don't get it either
import numpy as np

def main():
    M=16
    N=4
    #rad_v=np.mgrid()
    rad_v=((np.array(range(M+1))*2*np.pi)/M)
    #print(type(rad_v))
    #print(rad_v)
    cos_v=np.cos(rad_v)
    sin_v=np.sin(rad_v)
    
    print("rad_v",type(rad_v))
    
    print("sin_v",type(sin_v))
    #print(sin_v)
    
    n_grid,cos_grid=np.meshgrid(np.array(range(N)),np.array(cos_v))
    n_grid,sin_grid=np.meshgrid(np.array(range(N)),np.array(sin_v))
    print("cos_grid",type(cos_grid))
    print("sin_grid",type(sin_grid))
main()
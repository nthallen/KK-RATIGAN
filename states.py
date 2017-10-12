# Miles E. Allen, 12 October 2017

import copy

# A class to hold vital information regarding the Gondola.
class Gondola_State():
    current_position=(300,750,0)
    gondola_azimuth=0
    gondola_elevation=0
    gondola_speed=3
    
    def __init__(self,position,azimuth,elevation,speed):
        self.current_position=position
        self.gondola_azimuth=azimuth
        self.gondola_elevation=elevation
        self.gondola_speed=speed

    def copy(self):
        return copy.deepcopy(self)
    
    def get_position(self):
        return self.current_position
    
    def get_azimuth(self):
        return self.gondola_azimuth
    
    def get_elevation(self):
        return self.gondola_elevation
    
    def get_speed(self):
        return self.gondola_speed

# A class to hold information regarding the position of the gondola and its representative dots.
class Sphere_State():
    x=None
    y=None
    z=None
    
    def __init__(self,x,y,z):
        self.x=x
        self.y=y
        self.z=z
    
    def copy(self):
        return copy.deepcopy(self)

# A class to hold information regarding the vital information with which to draw lidar lines.
class Lidar_State():
    x=None
    y=None
    z=None
    t=None
    
    def __init__(self,x,y,z,t):
        self.x=x
        self.y=y
        self.z=z
        self.t=t
    
    def copy(self):
        return copy.deepcopy(self)
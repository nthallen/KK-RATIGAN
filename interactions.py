# Miles E. Allen, 12 October 2017

from pyface.qt import QtGui, QtCore
import queue
import random

# I needed a queue but I also needed it to be indexable ...
class IndexableQueue(queue.Queue):
    def __getitem__(self, index):
        with self.mutex:
            return self.queue[index]

# A class for the gondola's pause button.
class OffButton(QtGui.QPushButton):
    label="none"
    off=False
    gondola=None
    
    def __init__(self,string,gondola):
        QtGui.QPushButton.__init__(self,string)
        self.label=string
        self.off=False
        self.gondola=gondola

    def connect_released(self):
        self.released.connect(self.handle_released)

    def handle_released(self):
        self.on_or_off()

    def on_or_off(self):
        if self.off:
            self.off=False
            self.gondola.lidar_off=False
        else:
            self.off=True
            self.gondola.lidar_off=True

# A class for changing the view
class ViewButton(QtGui.QPushButton):
    label="none"
    gondola=None
    
    def __init__(self,string,gondola):
        QtGui.QPushButton.__init__(self,string)
        self.label=string
        self.gondola=gondola

    def connect_released(self):
        self.released.connect(self.handle_released)

    def handle_released(self):
        self.gondola.update_camera()

# A class for toggling the cloud's visibility
class CloudButton(QtGui.QPushButton):
    label="none"
    visible=True
    gondola=None
    
    def __init__(self,string,gondola):
        QtGui.QPushButton.__init__(self,string)
        self.label=string
        self.visible=True
        self.gondola=gondola

    def connect_released(self):
        self.released.connect(self.handle_released)

    def handle_released(self):
        self.appear_or_disappear()

    def appear_or_disappear(self):
        if self.visible:
            self.visible=False
            self.gondola.turn_cloud_off()
            self.setText("Cloud ON")
        else:
            self.visible=True
            self.gondola.turn_cloud_on()
            self.setText("Cloud OFF")

# A button to turn on the LIDAR graph.
class LidarGraphButton(QtGui.QPushButton):
    label="none"
    gondola=None

    def __init__(self,string,gondola):
        QtGui.QPushButton.__init__(self,string)
        self.label=string
        self.gondola=gondola
    
    def connect_released(self):
        self.released.connect(self.handle_released)
    
    def handle_released(self):
        self.gondola.graph_on=True
        self.gondola.lidar.handle_openings()

# A class for the gondola's pause button.
class PauseButton(QtGui.QPushButton):
    label="none"
    paused=False
    gondola=None
    
    def __init__(self,string,gondola):
        QtGui.QPushButton.__init__(self,string)
        self.label=string
        self.paused=False
        self.gondola=gondola

    def connect_released(self):
        self.released.connect(self.handle_released)

    def handle_released(self):
        self.pause_or_unpause()

    def pause_or_unpause(self):
        if self.paused:
            self.paused=False
            self.gondola.paused=False
            self.setText("Pause")
        else:
            self.paused=True
            self.gondola.paused=True
            self.setText("Play")

class CloudButton2(QtGui.QRadioButton):
    label="none"
    gondola=None
    visible=True
    
    def __init__(self,string,gondola):
        QtGui.QRadioButton.__init__(self,string,parent=None)
        self.label=string
        self.visible=True
        self.gondola=gondola

    def connect_released(self):
        self.released.connect(self.toggled)
        
    def toggled(self):
        if self.visible:
            self.visible=False
            self.gondola.turn_cloud_off()
            self.label="Cloud OFF"
        else:
            self.visible=True
            self.gondola.turn_cloud_on()
            self.label="Cloud ON"

class PauseButton2(QtGui.QRadioButton):
    label="none"
    gondola=None
    paused=False
    
    def __init__(self,string,gondola):
        QtGui.QRadioButton.__init__(self,string,parent=None)
        self.label=string
        self.paused=False
        self.gondola=gondola

    def connect_released(self):
        self.released.connect(self.toggled)
        
    def toggled(self):
        if self.paused:
            self.paused=False
            self.gondola.paused=False
            self.label="PLAYING"
        else:
            self.paused=True
            self.gondola.paused=True
            self.label="PAUSED"

# A class for the gondola's directional buttons.
class DirectionButton(QtGui.QPushButton):
    command_queue=None
    label = "none"
    direction=0
    gondola=None
    start=None
    
    def __init__(self,string,direction,gondola,command):
        QtGui.QPushButton.__init__(self,string)
        self.label=string
        self.direction=direction
        self.gondola=gondola
        self.command_queue=command
        
    def connect_released(self):
        self.released.connect(self.handle_released)
        
    def handle_released(self):
        self.start=self.gondola.return_time()
        self.command_queue.add(lambda: self.gondola.turn(self.direction))

# A class for the gondola's directional buttons.
class LidarScanDirectionButton(QtGui.QPushButton):
    command_queue=None
    label = "none"
    direction=0
    gondola=None
    start=None
    
    def __init__(self,string,direction,gondola,command):
        QtGui.QPushButton.__init__(self,string)
        self.label=string
        self.direction=direction
        self.gondola=gondola
        self.command_queue=command
        
    def connect_released(self):
        self.released.connect(self.handle_released)
        
    def handle_released(self):
        self.start=self.gondola.return_time()
        self.command_queue.add(lambda: self.gondola.turn_lidar_scan(self.direction))

# A class for the gondola's directional buttons.
class LidarDirectionButton(QtGui.QPushButton):
    command_queue=None
    label = "none"
    direction=0
    gondola=None
    start=None
    
    def __init__(self,string,direction,gondola,command):
        QtGui.QPushButton.__init__(self,string)
        self.label=string
        self.direction=direction
        self.gondola=gondola
        self.command_queue=command
        
    def connect_released(self):
        self.released.connect(self.handle_released)
        
    def handle_released(self):
        self.start=self.gondola.return_time()
        self.command_queue.add(lambda: self.gondola.turn_lidar(self.direction))

# A class for the slider that changes the speed of the gondola.
class SpeedSlider(QtGui.QSlider):
    speed_label=None
    command_queue=None
    value=3
    gondola=None
    def __init__(self,gondola,command):
        QtGui.QSlider.__init__(self, QtCore.Qt.Horizontal)
        QtGui.QSlider.setMinimum(self,0)
        QtGui.QSlider.setMaximum(self,3)
        QtGui.QSlider.setValue(self,3)
        self.gondola=gondola
        self.command_queue=command

    def connect_value_changed(self):
        self.valueChanged.connect(self.value_changed)
        
    def value_changed(self,value):
        self.value=value
        string="("+str(value)+")"
        self.speed_label.setText(string)
        self.command_queue.add(lambda: self.gondola.set_speed(value))

# A class for the slider that changes the speed of the gondola.
class AngleSlider(QtGui.QSlider):
    angle_label=None
    command_queue=None
    value=23
    gondola=None
    def __init__(self,gondola,command):
        QtGui.QSlider.__init__(self, QtCore.Qt.Horizontal)
        QtGui.QSlider.setMinimum(self,0)
        QtGui.QSlider.setMaximum(self,45)
        QtGui.QSlider.setValue(self,23)
        self.gondola=gondola
        self.command_queue=command

    def connect_value_changed(self):
        self.valueChanged.connect(self.value_changed)
        
    def value_changed(self,value):
        self.value=value
        if value<10:
            string="(0"+str(value)+")"
        else:
            string="("+str(value)+")"
        self.angle_label.setText(string)
        self.command_queue.add(lambda: self.gondola.set_angle(value))

# A class to hold the command queue to implement command latency.
class Command_Queue():
    delay=None
    c_queue=None
    gondola=None
    reliability=None

    def is_empty(self):
        if (len(self.c_queue)==0):
            return True
        else:
            return False

    def add_left(self,item):
        self.c_queue.appendleft(item)

    def add(self,item):
        chance=random.random()
        if (chance<self.reliability):
            self.c_queue.append((item,self.return_time()))

    def return_time(self):
        return self.gondola.return_time()

    def __init__(self,gondola,wait=0,rel=1):
        self.delay=wait
        self.gondola=gondola
        self.c_queue=queue.deque()
        self.reliability=rel
    
    def execute(self):
        return self.c_queue.popleft()
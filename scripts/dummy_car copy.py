#! /usr/bin/env python


#not sure if this script should be here 

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16


class Car(object):

    def __init__(self):
        self.speed=0.0
        self.steering_angle=0.0
        self.steering_torque=0.0
        self.throtle_pos=0.0
        self.brake_pos=0.0
        
        self.t_step=3

    def data_cb(self):
        pass

    def update_speed(self):
        pass
    def update_steering(self):
        pass 


#! /usr/bin/env python
import sys
import rospy
#from std_srvs.srv import SetBool
#not sure if this script should be here 

#from roscco.msg import EnableDisable
from roscco.msg import CanFrame, CanFrameData


from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Header

class Vehicle(object):
    def __init__(self):
        self.turn_state="S" # S , L, R
        self.speed_cmd=0.0
        self.steer_cmd=0.0
        self.brake_cmd=0.0
        self.turn_timer = None

THROTTLE_RATIO =(1.0/255) #  ~1/255 #const double THROTTLE_RATIO=1/255
THROTTLE_LIMIT = 0.4
THROTTLE_INCREMENT = (THROTTLE_RATIO*2)

BRAKE_LIMIT = 0.35 #1/780
BRAKE_LIMIT_SOFT = 0.02

STEERING_MAX = 5130
STEERING_RATIO =(1.0/STEERING_MAX)#~1/5200
STEERING_STATE_TOLERANCE = 0.15

SPEED_TOLERANCE = 0.5
SPEED_PANIC = 3.5 #if the speed error is bigger than this, soft brake

KM_TO_M = 0.621371

#if defined( KIA_SOUL_EV )
BRAKE_RATIO = 0.00115
SPEED_RATIO = 0.5

KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID = 0x2B0 
KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID = 0x4B0 
KIA_SOUL_OBD_BRAKE_PRESSURE_CAN_ID = 0x220 
KIA_SOUL_OBD_THROTTLE_PRESSURE_CAN_ID = 0x200 
KIA_SOUL_OBD_SPEED_CAN_ID = 0x4F2 
KIA_SOUL_OBD_STEERING_ANGLE_SCALAR = 0.1 

throttle_msg = Int16()
brake_msg = Int16()
angle_msg = Int16()
torque_msg = Int16()
speed_msg = Float32()

steering_angle_raw = 0 
steering_angle_raw_prev =0

def can_frame_callback( input):
    global steering_angle_raw,steering_angle_raw_prev
    publish_= True
    frame_id =input.frame.can_id 

    if frame_id == KIA_SOUL_OBD_THROTTLE_PRESSURE_CAN_ID: 
        throttle_raw = ord(input.frame.data[4])
        #throttle_report = throttle_raw * THROTTLE_RATIO
        #print(throttle_raw)
        throttle_msg.data = int(throttle_raw)

    elif frame_id ==  KIA_SOUL_OBD_BRAKE_PRESSURE_CAN_ID:
        
        brake_raw = ord(input.frame.data[4]) + ord(input.frame.data[5]) * 256
      
        #brake_report = brake_raw * BRAKE_RATIO
        brake_msg.data = int(brake_raw)

    elif frame_id ==  KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID: 
        steering_angle_raw_prev = steering_angle_raw
        steering_angle_raw = ord(input.frame.data[0]) + ord(input.frame.data[1]) * 256
        steering_torque_raw = ord(input.frame.data[2])
                    
        if steering_angle_raw > 27768:
            steering_angle_raw -= 65535
            
        if steering_angle_raw_prev > steering_angle_raw:
            steering_torque_raw=-steering_torque_raw
        #if (processed_last_)
        #steering_angle_report = steering_angle_raw #* STEERING_RATIO # [-1, queue_size = 1]
        # steering_angle_report = steering_angle_report

        angle_msg.data = int(steering_angle_raw)
        torque_msg.data = int(steering_torque_raw)


    elif frame_id == KIA_SOUL_OBD_SPEED_CAN_ID:
        speed_report = ord(input.frame.data[1])
        speed_report = speed_report * SPEED_RATIO
        speed_msg.data=speed_report

    else:
        publish_ = False

    if publish_:
        obd2_speed_pub.publish(speed_msg)
        obd2_throttle_pub.publish(throttle_msg)
        obd2_brake_pub.publish(brake_msg)
        obd2_steer_angle_pub.publish(angle_msg)
        obd2_steer_torque_pub.publish(torque_msg)
 

if __name__ == '__main__':
    #add /raw to msgs
    rospy.init_node('odb_report_parser', anonymous=True)
        
    obd2_speed_pub = rospy.Publisher("oscc/speed", Float32, queue_size=1)
    obd2_throttle_pub =  rospy.Publisher("oscc/throttle",  Int16, queue_size = 1)
    obd2_brake_pub =  rospy.Publisher("oscc/brake",  Int16, queue_size = 1)
    obd2_steer_angle_pub =  rospy.Publisher("oscc/steering",  Int16, queue_size = 1)
    obd2_steer_torque_pub =  rospy.Publisher("oscc/steering_torque",  Int16, queue_size = 1)

    rospy.Subscriber("/can_frame", CanFrame, can_frame_callback)


    rospy.loginfo("Can frame parser node started")


    #rate = rospy.Rate(RATE) # 15hz
     
    rospy.spin()

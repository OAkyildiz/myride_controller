#!/usr/bin/env python
# license removed for brevity
import rospy
import yaml
from rosbag.bag import Bag

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDriveStamped

CRUISE_SPEED = 8
RATE=2
from threading import Timer

def get_topics():
    bag = Bag('input.bag')
    topics = bag.get_type_and_topic_info()[1].keys()
    types = []
    for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])

def get_bag_info():
    info_dict = yaml.load(Bag('input.bag', 'r')._get_yaml_info())

class Commander(object):
    def __init__(self):
        self.state="stop" #TODO: do Enum
        self.turn_state="S" # S , L, R
        self.speed_cmd=0.0
        self.steer_cmd=0.0
        self.brake_cmd=0.0
        self.turn_timer = None


    def command_callback(self,msg):
        cmd=msg.data
        rospy.loginfo(rospy.get_caller_id() + " recevied %s", cmd)

        #received "start" set speed to 15.0
        #received "stop" desired speed=0, throttle off
        #received "park" for now: do stop.
        self.state=cmd
        if cmd == "start":
            self.speed_cmd=CRUISE_SPEED
        elif cmd == "stop":
            self.speed_cmd=0.0
        elif cmd == "park":
            self.speed_cmd=-1.0
        else:
            print("unknown command")

    def turn_callback(self, msg):
        self.turn_timer = Timer(6.5,self.correct_steer)
        self.turn_timer.start()
        self.steer_cmd=msg.data
        self.turn_state="T"+str(msg.data)

    def correct_steer(self):
        self.steer_cmd=0.0
        self.turn_state="S"

    def ackermann_callback(self, ack_msg):
        full_cmd = ack_msg.drive #it also contains steering_velocity, accel, jerk
        self.speed_cmd = full_cmd.speed
        self.steer_cmd = full_cmd.steering_angle


    def init(self):
        # speed_pub = rospy.Publisher('oscc/speed', Float32, queue_size=10)
        # steer_pub = rospy.Publisher('oscc/steering', Int16, queue_size=10)
        # steer_pub = rospy.Publisher('oscc/brake', Int16, queue_size=10)

        self.speed_pub = rospy.Publisher('/navigation/target/speed', Float32, queue_size=10)
        self.steering_pub = rospy.Publisher('/navigation/target/steering', Float32, queue_size=10)
        self.brake_pub = rospy.Publisher('/navigation/target/brake', Int16, queue_size=10)

        rospy.Subscriber("command", String, self.command_callback)
        rospy.Subscriber("turn_command", Float32, self.turn_callback)

        rospy.Subscriber("ackermann_command", AckermannDriveStamped, self.ackermann_callback)
    
    def loop(self):
        rospy.init_node('demo_controller', anonymous=True)
        rospy.loginfo("Demo controller node started")
        rate = rospy.Rate(RATE) # 15hz

       
            

        while not rospy.is_shutdown():
        
            self.speed_pub.publish(Float32(self.speed_cmd))
            self.steering_pub.publish(Float32(self.steer_cmd))
            #self.brake_pub.publish(Int16(0))

            rate.sleep()

if __name__ == '__main__':
    node=Commander()
    node.init()
    try:
        node.loop()
    except rospy.ROSInterruptException:
        pass

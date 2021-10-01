#! /usr/bin/env python
import sys
import rospy
from std_srvs.srv import SetBool
#not sure if this script should be here 
from roscco.msg import ThrottleCommand
from roscco.msg import BrakeCommand
from roscco.msg import SteeringCommand
from roscco.msg import SteeringAngleCommand
from roscco.msg import EnableDisable


from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Header

BRAKE_SOFT = 0.17
BRAKE_FULL = 0.35
BRAKE_EXP = 1.05

FROM_RAW=0
FROM_PERCENTAGE=1
FROM_REAL=2

THROTTLE_DIV = {FROM_RAW:255.0,FROM_PERCENTAGE:100.0}
BRAKE_DIV = {FROM_RAW:500.0,FROM_PERCENTAGE:100.0}
STEERING_TORQUE_DIV = {FROM_RAW:270.0,FROM_PERCENTAGE:100.0}
STEERING_ANGLE_DIV = {FROM_RAW:5250.0,FROM_REAL:10.0}

class Controller(object):
    name="driving_playback_torque"
    

    def __init__(self, playername =''):
        self.player_name = playername
        self.throttle_cmd = ThrottleCommand()
        self.throttle_cmd.header=self.create_header()
        
        self.brake_cmd = BrakeCommand()
        self.brake_cmd.header=self.create_header()

        self.steering_torque_cmd = SteeringCommand() #will be deprecated
        self.steering_torque_cmd.header=self.create_header()

        self.prev_angle=0.0
        self.torque_sign=1.0

        self.steering_angle_cmd = SteeringAngleCommand()
        self.steering_angle_cmd.header=self.create_header()

        self.enable_disable_msg = EnableDisable()
        self.enable_disable_msg.header=self.create_header()
        self.enable_disable_msg.enable_control=False
        #self.data_interface = RawToCommand()
        self.data_type=FROM_RAW

    def create_header(self):
        hdr = Header()
        #hdr.stamp = rospy.Time.now()
        hdr.frame_id="controller"
        return hdr

    def node_init(self):
        rospy.init_node(self.name)
        rospy.loginfo("Driving data playback node started")


        self.throttle_pub = rospy.Publisher('/throttle_command', ThrottleCommand, queue_size=5)
        self.brake_pub = rospy.Publisher('/brake_command', BrakeCommand, queue_size=5)
        self.steering_torque_pub = rospy.Publisher('/steering_torque_command', SteeringCommand, queue_size=5)
        #self.steering_angle_pub = rospy.Publisher('/steering_angle_command', SteeringAngleCommand, queue_size=5)
        self.enable_disable_pub = rospy.Publisher('/enable_disable', EnableDisable, queue_size=5)

        #wait for player service available
        if (self.player_name): 
            rospy.loginfo("Interacting with " + self.player_name)
            srv = '/'+self.player_name+'/pause_playback'
            rospy.wait_for_service(srv)
            self.playpause_srv = rospy.ServiceProxy(srv, SetBool)
        else:
            rospy.loginfo("No player attached")

        rospy.Subscriber("/oscc/throttle", Int16, self.update_throttle)
        rospy.Subscriber("/oscc/brake", Int16, self.update_brake)
        rospy.Subscriber("/oscc/steering", Int16, self.update_steering_angle) # self.update_wheel
        rospy.Subscriber("/oscc/steering_torque", Int16, self.update_steering_torque)
        rospy.Subscriber("/oscc/speed", Float32, self.update_speed)
        rospy.Subscriber('/command', String, self.cmd_cb)



    #make an update() method factory

    #recorder playpause
    def playpause_req(self, btn):
        if (self.player_name):
            try:
                return self.playpause_srv(btn)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        else: 
            return False

    ## Driving Commands
    def cmd_cb(self,cmd_msg):
        cmd=cmd_msg.data.lower()
        rospy.loginfo("received "+cmd)

        if  cmd == "start":
            self.start()

        elif  cmd == "stop":
            self.stop()        

        elif  cmd == "park":
            self.park() 

        else:
            rospy.logerr("Unexpected command")

    def start(self):
        self.enable()
        self.playpause_req(False)

    def stop(self):
        if not self.enable_disable_msg.enable_control:
            self.enable()
            self.send_throttle(0.0)
            self.send_brake(BRAKE_FULL)
        else:
            self.playpause_req(True)
            self.brake()

    def park(self):
        #pause service
        self.playpause_req(True)
        # parking routine
        self.disable()

    def brake(self): #make init and factor dependent on speed, hence the braking time
        brake_position = 0.05 #1/2/SPEED
        while brake_position < BRAKE_FULL:
            # impl send_brake() , send_X()
            brake_position*=BRAKE_EXP
            
            self.send_throttle(0.0)
            self.send_brake(brake_position)
            rospy.sleep(0.08)
            
        self.send_throttle(0.0)
        self.send_brake(BRAKE_FULL)

    #repeptetive make DIV an arg
    def send_brake(self, brk):
        self.brake_cmd.header.stamp = rospy.Time.now()
        self.brake_cmd.brake_position = brk
        self.brake_pub.publish(self.brake_cmd)
    
    def send_throttle(self, thr):
        self.throttle_cmd.header.stamp = rospy.Time.now() #update_stamp() ?
        self.throttle_cmd.throttle_position = thr
        self.throttle_pub.publish(self.throttle_cmd)

    ## Enable Disable
    def enable(self):
        #enable
        self.enable_disable_msg.header.stamp=rospy.Time.now()
        self.enable_disable_msg.enable_control=True
        self.enable_disable_pub.publish(self.enable_disable_msg)
        #brake
        self.send_throttle(0.0)
        self.send_brake(BRAKE_SOFT)

    def disable(self):
        #brake first
        self.send_throttle(0.0)
        self.send_brake(BRAKE_SOFT)
        #disable
        self.enable_disable_msg.header.stamp=rospy.Time.now()
        self.enable_disable_msg.enable_control=False
        self.enable_disable_pub.publish(self.enable_disable_msg)
        
    ## Driving Data
    def update_throttle(self, throttle):
        self.throttle_cmd.header.stamp = rospy.Time.now() #update_stamp() ?
        self.throttle_cmd.throttle_position = throttle.data / THROTTLE_DIV[self.data_type]
        
        self.throttle_pub.publish(self.throttle_cmd)


    def update_brake(self, brake):
        self.brake_cmd.header.stamp = rospy.Time.now()
        self.brake_cmd.brake_position = brake.data / BRAKE_DIV[self.data_type]
        
        self.brake_pub.publish(self.brake_cmd)


    def update_steering_torque(self, torque):
        self.steering_torque_cmd.header.stamp = rospy.Time.now()
        self.torque_sign=(1.0 if self.steering_angle_cmd.steering_angle >= self.prev_angle else -1.0)

        self.steering_torque_cmd.steering_torque = abs(torque.data / STEERING_TORQUE_DIV[self.data_type])*self.torque_sign
        self.steering_torque_pub.publish(self.steering_torque_cmd)


    def update_steering_angle(self, angle):
        self.steering_angle_cmd.header.stamp = rospy.Time.now()
        self.prev_angle=self.steering_angle_cmd.steering_angle
        #self.angle=
        self.steering_angle_cmd.steering_angle = angle.data / STEERING_ANGLE_DIV[self.data_type]
        
        #self.steering_angle_pub.publish(self.steering_angle_cmd)


    def update_speed(self, speed):
        self.speed=speed.data


if __name__ == '__main__':
    #rosbag playername
    args=sys.argv
    print(args)
    if len(args) > 3: 
        player=args[1]    
    else: #check if args[1]   is __name
        player=''
    
    ctrl=Controller(player)
    ctrl.node_init()
    #3 callback queue?
    ctrl.enable()
    stop_msg=Float32(data=0)
    ctrl.update_brake(stop_msg)
    rospy.spin()
    ctrl.update_brake(stop_msg)
    ctrl.disable()
    print("Exiting...")
# class RawToCommand(object): #TODO: Base Class DataInterface

#     THROTTLE_TO_CMD=255.0
#     BRAKE_TO_CMD=787.0
#     STEERING_ANGLE_TO_CMD=10.0
#     name = "raw"

#     def __init__(self):

    

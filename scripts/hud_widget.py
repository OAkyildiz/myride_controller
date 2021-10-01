#! /usr/bin/env python

import cv2
import numpy as np
import sys

RATE = 40
ROS_ENABLED = 0


args=sys.argv
argc=len(args)    
#TODO: don't import if argv[2]

       
  
if argc == 1:
    import rospy
    from std_msgs.msg import Float32
    from std_msgs.msg import Int16
    ROS_ENABLED = 1


PI=np.pi
STEERING_ABS_MAX = 5130
STEERING_FACTOR = 10
STEERING_TORQUE_MAX = 64

BRAKE_MAX = 780
THROTTLE_MAX = 255

FONT = cv2.FONT_HERSHEY_SIMPLEX 


def bits_to_angle(bt):
    return np.radians(float(bt)/STEERING_FACTOR)

class HUD():
    WINDOW_NAME = "HUD"
    #slots for this
    ratio_widget = 0.60 
    ratio_padding = 0.125 #8 * 2
    ratio_margin = ratio_padding/4 #2
    
    ratio_gage = ratio_padding*0.3

    ratio_extension = 1-2*ratio_padding-ratio_margin - ratio_widget
    
    
    ratio_left= ratio_gage*2+ratio_padding + 3*ratio_margin

    GRAY_WHEEL = 120
    GRAY_WHEEL_TEXT = 155
    GRAY_TORQUE= 50
    GRAY_BRAKE = 75
    GRAY_THROTTLE= 180
    GRAY_SPEED= 170

    COLOR_WHEEL = (155, 75, 85)
    COLOR_WHEEL_TEXT = (180, 50, 50)
    COLOR_TORQUE= (60,120,120)
    COLOR_BRAKE = (20, 20, 140)
    COLOR_THROTTLE= (60,160,60)
    COLOR_SPEED= (70,150,70)

    def __init__(self, size=80, color=0):
        self.color=color
        self.colors=({'wheel': self.COLOR_WHEEL, 'wheel_text': self.COLOR_WHEEL_TEXT, 'wheel_torque': self.COLOR_TORQUE, 'brake': self.COLOR_BRAKE,'throttle': self.COLOR_THROTTLE, 'speed': self.COLOR_SPEED} \
        if color else {'wheel': self.GRAY_WHEEL, 'wheel_text': self.GRAY_WHEEL_TEXT, 'wheel_torque': self.GRAY_TORQUE, 'brake': self.GRAY_BRAKE,'throttle': self.GRAY_THROTTLE, 'speed': self.GRAY_SPEED})
        self.gage={}
        self.torque_raw=0
        self.angle_raw=0
        self.brake_raw=0
        self.throttle_raw=0
        self.speed=0.0
        self.resize(size)

        if __name__ == '__main__':
            cv2.namedWindow(self.WINDOW_NAME)

    def resize(self,size):
        #set all calculate-once dimensions
        
      
        w = int(size * (1 + self.ratio_left))
        h = int(size)

   
        self.pad = int(size * self.ratio_padding)
        gage_w = int(size * self.ratio_gage)
        margin = int(size * self.ratio_margin)
        
       
        
        self.wheel_radius = int(size * self.ratio_widget)//2
        self.wheel_center = self.wheel_radius+self.pad


        gage={}
        
        gage['Y'] = self.wheel_center + self.wheel_radius
        gage['Y_MAX_D'] = int(size * self.ratio_widget) #self.wheel_radius*2
        gage['1_X1'] = size + margin
        gage['1_X2'] = gage['1_X1']  + gage_w
        gage['2_X1'] = gage['1_X2'] + margin
        gage['2_X2'] = gage['2_X1']  + gage_w
        
        text_h=22 #getTextSize default
        gage['text_scale'] =  gage_w*2/22 #not redundant, tying just scale to gage_w 
        
        textSize = cv2.getTextSize(' ', fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=gage['text_scale'] , thickness=1)
                
        text_h = textSize[0][1]
        text_w = textSize[0][0]

        print(text_w,text_h)
        print('m', margin)
        gage['wheel_text_L'] = 8
        gage['wheel_text_X'] = self.wheel_center - int((gage['wheel_text_L'] * text_w) //2)
        gage['wheel_text_Y'] = self.wheel_center + text_h + margin
        
        gage['torque_X'] = self.wheel_center #redundant
        gage['torque_MAX_D'] = self.wheel_radius 
        gage['torque_Y2'] = size - self.pad
        gage['torque_Y1'] = gage['torque_Y2'] - gage_w
       

        gage['speed_L'] = 7
        gage['speed_X'] = gage['2_X2'] - int(gage['speed_L'] * text_w)
        gage['speed_Y'] = gage['Y'] + text_h + margin
        
        self.gage=gage

        #draw the template

       
        mat=((h, w, 3) if self.color else (h, w))
        self.canvas=np.full(mat,0,dtype='uint8')

        self.canvas = cv2.circle(self.canvas, (self.wheel_center, self.wheel_center), self.wheel_radius, self.colors['wheel'], 3)

    #TODO: add steering torque under the wheel, bi-directional
    #TODO: Add speed


    def draw(self):
        colors=self.colors
        g_dims=self.gage #gage diomensions
        hud_image = self.canvas.copy()

        #draw Wsteering angle indicator
        center=(self.wheel_center,  self.wheel_center)
        rads=bits_to_angle(self.angle_raw)
        dir_x = int(np.around(-np.sin(rads),decimals=6) * self.wheel_radius + self.wheel_center)
        dir_y = int(np.around(-np.cos(rads),decimals=6) * self.wheel_radius + self.wheel_center)
        cv2.line(hud_image, center, (dir_x, dir_y), colors['wheel'], 3)
        #TODO: Parameterize dimensions
        #print angle on hud
        cv2.putText(hud_image, str(self.angle_raw/10).rjust(g_dims['wheel_text_L'],' '), (g_dims['wheel_text_X'],  g_dims['wheel_text_Y']), FONT, \
                   g_dims['text_scale'], colors['wheel_text'], 1, cv2.LINE_AA, ) 
        #draw torque
        torque_w = int(g_dims['torque_X'] - (self.torque_raw *  g_dims['torque_MAX_D'])//STEERING_TORQUE_MAX)

        cv2.rectangle(hud_image, (g_dims['torque_X'], g_dims['torque_Y1']), (torque_w, g_dims['torque_Y2']), colors['wheel_torque'], -1)

        #print(dir_x, dir_y)

        #draw brake and throttle gages
        brake_h = (self.brake_raw*g_dims['Y_MAX_D']) // BRAKE_MAX  #draw a horz line for base/0
        throttle_h = (self.throttle_raw*g_dims['Y_MAX_D'])// THROTTLE_MAX
        cv2.rectangle(hud_image, (g_dims['1_X1'], g_dims['Y']), (g_dims['1_X2'], g_dims['Y']-brake_h), colors['brake'], -1)
        cv2.rectangle(hud_image, (g_dims['2_X1'], g_dims['Y']), (g_dims['2_X2'], g_dims['Y']-throttle_h), colors['throttle'],-1)
        # print(brake_h, throttle_h)
       
        # speed
        cv2.putText(hud_image, (str(self.speed)+ ' mph').rjust(g_dims['speed_L'],' '), (g_dims['speed_X'],  g_dims['speed_Y']), FONT, \
                   g_dims['text_scale'], colors['speed'], 1, cv2.LINE_AA ) 
        print( "{} deg. {} mph".format(int(self.angle_raw/10), self.speed))
        return hud_image

        
    def update_wheel(self,angle):
        self.angle_raw=angle.data
    
    def update_brake(self,brake):
        self.brake_raw=brake.data

    def update_throttle(self, throttle):
        self.throttle_raw=throttle.data
    
    def update_steering_torque(self, torque):
        self.torque_raw=torque.data
    
   
    def update_speed(self, speed):
        self.speed=speed.data

    #also a test
    if __name__ == '__main__':
        def show(self):
            cv2.imshow(self.WINDOW_NAME,self.draw())

        def show_widget(self, angle, brake, throttle):
            self.update_wheel(angle)
            self.update_brake(brake)
            self.update_throttle(throttle)
            self.show()

        def HUD_exit(self):
            cv2.destroyWindow(self.WINDOW_NAME) 
            sys.exit()
    else:
        def set_widget(self, angle, brake, throttle):
            self.update_wheel(angle)
            self.update_brake(brake)
            self.update_throttle(throttle)
            return self.draw() 

   

    if ROS_ENABLED:
        def HUD_init(self):
            if __name__ == '__main__':
                rospy.init_node("wheel_node")
                
            rospy.Subscriber("/oscc/steering", Int16, self.update_wheel)
            rospy.Subscriber("/oscc/brake", Int16, self.update_brake)
            rospy.Subscriber("/oscc/throttle", Int16, self.update_throttle)
            rospy.Subscriber("/oscc/steering_torque", Int16, self.update_steering_torque)
            rospy.Subscriber("/oscc/speed", Float32, self.update_speed)
            rospy.loginfo("HUD node started")
        
        if __name__ == '__main__':
            def HUD_spin(self):
                
                #r = rospy.Rate(RATE)
                while not rospy.is_shutdown():
                    # if the node is headlass we should not need a loop, as it will be blocking
                    self.show()
                    if cv2.waitKey(1000//RATE) & 0xFF == ord('q'):
                        break
    else:
        if __name__ == '__main__':
            def HUD_spin(self):
                while True:
                    self.show()
                    if cv2.waitKey(1000//RATE) & 0xFF == ord('q'):
                        break

   
    
#TODO: module
if __name__ == '__main__':
    
    hud=HUD(120,color = True)
    if argc == 1:
        hud.HUD_init()
        hud.HUD_spin()

    elif argc == 2:
        hud.update_brake(50)
        hud.update_wheel(100)

        hud.HUD_spin()

    elif argc == 4:
        hud.show_widget(int(args[1]), int(args[2]), int(args[3]))     
        cv2.waitKey()
    else:
       sys.exit("Bad input!")
       
    hud.HUD_exit()
        
   ###

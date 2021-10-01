#!/usr/bin/env python

import tkinter as tk
import rospy, sys

from std_msgs.msg import String
from std_msgs.msg import Float32

class GUI(tk.Frame, object):
    def __init__(self, master=None):
        super(GUI, self).__init__(master)
        self.master = master
        self.pack()
        self.create_widgets()
        self.cmd_pub = rospy.Publisher('/command', String, queue_size=10)
        self.turn_pub = rospy.Publisher('/turn_command', Float32, queue_size=10)
        self.angle = 0.55

    def create_widgets(self):

        self.left_btn = tk.Button(self)
        self.left_btn["command"] = self.turn_left
        self.left_btn["text"] = "<-"
        self.left_btn.pack(side="left")

        self.right_btn = tk.Button(self)
        self.right_btn["command"] = self.turn_right
        self.right_btn["text"] = "->"
        self.right_btn.pack(side="right")

        self.go_btn = tk.Button(self)
        self.go_btn["command"] = self.go
        self.go_btn["text"] = " GO "
        self.go_btn["fg"] = "green"
        self.go_btn.pack(side="top")

        self.stop_btn = tk.Button(self)
        self.stop_btn["command"] = self.stop
        self.stop_btn["text"] = "Stop"
        self.stop_btn["fg"] = "orange"
        self.stop_btn.pack(side="top")

        self.park_btn = tk.Button(self)
        self.park_btn["command"] = self.park
        self.park_btn["text"] = "Park"
        self.park_btn["fg"] = "red"
        self.park_btn.pack(side="top")

        self.quit_btn = tk.Button(self, text="[QUIT]", fg="red",
                              command=self.quit)

        self.quit_btn.pack(side="bottom")


    def quit(self):
        self.cmd_pub.publish(String("park"))
        self.master.destroy()
        

    
    def go(self):
        self.cmd_pub.publish(String("start"))

    def stop(self):
        self.cmd_pub.publish(String("stop")) 

    def park(self):
        self.cmd_pub.publish(String("park"))

    def turn_right(self):
        self.turn_pub.publish(Float32(-1*self.angle))
    
    def turn_left(self):
        self.turn_pub.publish(Float32(self.angle))

root = tk.Tk() 
rospy.init_node('gui', anonymous=True)
app = GUI(master=root)
app.mainloop()
sys.exit(app.quit())
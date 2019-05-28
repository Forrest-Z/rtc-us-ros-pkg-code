#!/usr/bin/env python                                                                                        
# Software License Agreement (BSD License)                                                                   
#                                                                                                            
# Copyright (c) 2009, Willow Garage, Inc.                                                                    
# All rights reserved.                                                                                       
#                                                                                                            
# Redistribution and use in source and binary forms, with or without                                         
# modification, are permitted provided that the following conditions                                         
# are met:                                                                                                   
#                      
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided                                         
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived                                         
#    from this software without specific prior written permission.
#                                                                                                            
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS                                          
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT                                         
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE                                            
# POSSIBILITY OF SUCH DAMAGE.
#
# Author Melonee Wise 

import roslib; roslib.load_manifest('rtcus_erratic_slam')
import rospy
import joy
import geometry_msgs

#erratic commands
from geometry_msgs import msg
import std_msgs
from std_msgs import msg

#joystick
from joy import msg

#voice syntetizer
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


import teleop_gui
import threading
import wx


class rtcus_erratic_teleop:
    def __init__(self):
      self.name ='rtcus_erratic_teleop'
      rospy.init_node(self.name)

      self.soundhandle = SoundClient()
      self.mode=0

      self.linear = rospy.get_param('axis_linear', 1)
      self.angular = rospy.get_param('axis_angular', 2)
      self.l_scale = rospy.get_param('scale_linear', 50.0)
      self.a_scale = rospy.get_param('scale_angular', 2.0)
    
      self.pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist)
      self.pub_ranger_tilt= rospy.Publisher("cmd_ranger_tilt",std_msgs.msg.Float64)
      self.sub = rospy.Subscriber("joy", joy.msg.Joy, self.on_joy_message)
      self.lock = threading.RLock()
              
    def on_joy_message(self,joy):
      vel = geometry_msgs.msg.Twist()
      vel.angular.z = self.a_scale*joy.axes[0];
      vel.linear.x = joy.axes[1];  
      
      
      laser_tilt_angle=joy.axes[2]*0.95 +(1-joy.axes[2])*(-1.5);
      self.pub_ranger_tilt.publish(std_msgs.msg.Float64(laser_tilt_angle))
      rospy.loginfo(rospy.get_caller_id()+"I heard %s",laser_tilt_angle)
      
      self.pub.publish(vel);
      if(joy.buttons[0]==1):
        rospy.loginfo("saying: %s")
        self.soundhandle.say("Hello")
        
    def ros_main_loop(self):
      rospy.spin();
      
    def say_message(self,text):
      self.soundhandle.say(text);
    

if __name__ == '__main__':
  teleopFacade = rtcus_erratic_teleop();
  
  ros_thread = threading.Thread(target=teleopFacade.ros_main_loop)
  ros_thread.start()
  
  app = wx.App(False)
  frame = wx.Frame(None)
  frame.Size=(600,600)
  frame.Title="erratic teleop"
  panel = teleop_gui.ErraticTeleopPanel(frame,teleopFacade);
  frame.Show()
  app.MainLoop()
  
  

  
  
#!/usr/bin/env python 
import roslib
roslib.load_manifest('rtcus_kinect_gestures')
import rospy
import tf
from plot_rviz_utils import *


rospy.init_node("tf_restamper")
tfpublisher= rospy.Publisher("tf",tf.msg.tfMessage)

def tfcallback(tfmessage):
    for transf in tfmessage.transforms:
        transf.header.stamp=rospy.Time.now()
        
        vector=numpy.matrix([transf.transform.translation.x,transf.transform.translation.y,transf.transform.translation.z])
        markerVector(transf.header.frame_id,"1000",vector,position=None,namespace="manequim")
        
    tfpublisher.publish(tfmessage)
tfproxy = rospy.Subscriber("tf_input",tf.msg.tfMessage,tfcallback)
rospy.spin()
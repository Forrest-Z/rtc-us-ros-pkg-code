#!/usr/bin/python
import roslib
roslib.load_manifest("rtcus_navigation")
import rospy
import std_msgs.msg
import nav_msgs
from nav_msgs.msg import Odometry
import numpy
from abc import abstractmethod
import rtcus_nav_msgs
import tf

from rtcus_nav_msgs.msg import DynamicState2DStamped

class PredictionErrorStatsMonitor():
    def __init__(self):
        self.estimation_sub_ = rospy.Subscriber("prediction_monitor/state_estimation_input", DynamicState2DStamped, self.estimation_callback);
        self.ground_truth_sub_ = rospy.Subscriber("prediction_monitor/ground_truth_input", Odometry, self.ground_truth_callback);
        self.ground_truth_buffer = []
        self.estimation_buffer = []
        
        self.distance_error = []
        self.orientation_errors = []
        self.d_linear_errors = []
        self.d_angular_errors = []
        
        #this represent epsilon_l in the PTC Paper
        self.distance_error_pub = rospy.Publisher("/prediction_quality/distance_error", std_msgs.msg.Float32)
        
        #this represent epsilon_fi in the PTC Paper
        self.orientation_error_pub = rospy.Publisher("/prediction_quality/orientation_error", std_msgs.msg.Float32)
        
        #this represent d[epsilon_l]/dt in the PTC Paper
        self.linear_error_pub = rospy.Publisher("/prediction_quality/linear_speed_error", std_msgs.msg.Float32)
        
        #this represent d[epsilon_fi]/dt in the PTC Paper
        self.angular_error_pub = rospy.Publisher("/prediction_quality/angular_speed_error", std_msgs.msg.Float32)
        rospy.logwarn("This implementation is just for non-holonomic robot, see converOdomToDynamicState.")
    
    def restart(self):
        self.ground_truth_buffer = []
        self.distance_error = []
        self.orientation_errors = []
        self.d_linear_errors = []
        self.d_angular_errors = []

    def convertOdomToDynamicState(self, odom):
        ret = DynamicState2DStamped()
        ret.header = odom.header
        ret.state.pose.x = odom.pose.pose.position.x
        ret.state.pose.y = odom.pose.pose.position.y
        ret.state.pose.phi = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])[2]
        #JUST FOR NONHOLONOMIC ROBOTS
        ret.state.twist.linear = numpy.sqrt(odom.twist.twist.linear.x * odom.twist.twist.linear.x + odom.twist.twist.linear.y * odom.twist.twist.linear.y)
        ret.state.twist.lateral = 0
        ret.state.twist.angular = odom.twist.twist.angular.z
        return ret

    def estimation_callback(self, estimation):
        #rospy.loginfo("NEW ESTIMATION at %lf", estimation.header.stamp.to_sec())
        selected_ground_truth_index = None
        #find last applicable estimation
        self.estimation_buffer.append(estimation)
        self.check()

    def ground_truth_callback(self, current_ground_truth):
        self.ground_truth_buffer.append(current_ground_truth)
        
    def check (self):
        while len(self.ground_truth_buffer) > 1 and len(self.estimation_buffer) > 0:
            estimation = self.estimation_buffer[0]
            if estimation.header.stamp > self.ground_truth_buffer[-1].header.stamp:
                break;
            elif(estimation.header.stamp < self.ground_truth_buffer[0].header.stamp):
                #pop this and others old estimation
                self.estimation_buffer = self.estimation_buffer[1:-1]
            else:
                selected_ground_truth_index = None
                #rospy.loginfo("ok there is data")
                for i in xrange(len(self.ground_truth_buffer) - 1) :
                    ground_truth = self.ground_truth_buffer[i]
                    next_ground_truth = self.ground_truth_buffer[i + 1]
                    #rospy.loginfo("cheking ground trugh %d: stamp %lf", i, ground_truth.header.stamp.to_sec())
                    if ground_truth.header.stamp <= estimation.header.stamp and next_ground_truth.header.stamp >= estimation.header.stamp:
                        selected_ground_truth_index = i;
                        break
                    
                #rospy.loginfo("Data count now: %d", len(self.ground_truth_buffer))
                #rospy.loginfo ("looking for ground truth with stamp == (^x(t_u)): %lf", estimation.header.stamp.to_sec())
                #rospy.loginfo ("first ground truth stamp: %lf", self.ground_truth_buffer[0].header.stamp.to_sec())
                #rospy.loginfo ("last ground truth stamp: %lf", self.ground_truth_buffer[-1].header.stamp.to_sec())
                if selected_ground_truth_index != None:
                    #rospy.loginfo("Ok. selected ground truth: %d", len(self.ground_truth_buffer))
                    #rospy.loginfo ("selected ground truth stamp: %lf", self.ground_truth_buffer[selected_ground_truth_index].header.stamp.to_sec())
                    #rospy.loginfo ("selected next ground truth stamp: %lf", self.ground_truth_buffer[selected_ground_truth_index+1].header.stamp.to_sec())
                
                    ground_truth = self.ground_truth_buffer[selected_ground_truth_index]
                    next_ground_truth = self.ground_truth_buffer[selected_ground_truth_index + 1]
                    #rospy.loginfo("first ground_truth: %lf",ground_truth.header.stamp.to_sec())
                    #rospy.loginfo("second ground_truth: %lf",next_ground_truth.header. stamp.to_sec())
                    
                    interpolated_ground_truth = None
                    if ground_truth.header.stamp == estimation.header.stamp:
                        interpolated_ground_truth = self.convertOdomToDynamicState(ground_truth)
                    elif next_ground_truth.header.stamp == estimation.header.stamp:
                        interpolated_ground_truth = self.convertOdomToDynamicState(next_ground_truth)
                    else:
                        #rospy.loginfo("INTERPOLATING GROUND TRUTH at %lf between %lf and %lf", estimation.header.stamp.to_sec(), ground_truth.header.stamp.to_sec(), next_ground_truth.header.stamp.to_sec())
                        weight = (estimation.header.stamp - ground_truth.header.stamp).to_sec() / (next_ground_truth.header.stamp - ground_truth.header.stamp).to_sec()
                        #rospy.loginfo("weight got: %lf", weight)
                        interpolated_ground_truth = DynamicState2DStamped()
                        interpolated_ground_truth.state.pose.x = weight * ground_truth.pose.pose.position.x + (1.0 - weight) * next_ground_truth.pose.pose.position.x
                        interpolated_ground_truth.state.pose.y = weight * ground_truth.pose.pose.position.y + (1.0 - weight) * next_ground_truth.pose.pose.position.y
                        last_ground_orientation = [ground_truth.pose.pose.orientation.x, ground_truth.pose.pose.orientation.y, ground_truth.pose.pose.orientation.z, ground_truth.pose.pose.orientation.w]
                        next_ground_truthorientation = [ground_truth.pose.pose.orientation.x, ground_truth.pose.pose.orientation.y, ground_truth.pose.pose.orientation.z, ground_truth.pose.pose.orientation.w]
                        interpolated_ground_truth.state.pose.phi = weight * tf.transformations.euler_from_quaternion(last_ground_orientation)[2] + (1.0 - weight) * tf.transformations.euler_from_quaternion(next_ground_truthorientation)[2]
                        
                        interpolated_ground_truth.state.twist.linear = weight * numpy.sqrt(ground_truth.twist.twist.linear.x * ground_truth.twist.twist.linear.x + ground_truth.twist.twist.linear.y * ground_truth.twist.twist.linear.y) + (1.0 - weight) * numpy.sqrt(next_ground_truth.twist.twist.linear.x * next_ground_truth.twist.twist.linear.x + next_ground_truth.twist.twist.linear.y * next_ground_truth.twist.twist.linear.y)
                        interpolated_ground_truth.state.twist.angular = weight * ground_truth.twist.twist.angular.z + (1.0 - weight) * next_ground_truth.twist.twist.angular.z
                
                    error_value_msg = std_msgs.msg.Float32()
                    
                    #LINEAR DISTANCE WORKSPACE ERROR
                    distance_error = numpy.sqrt((estimation.state.pose.x - interpolated_ground_truth.state.pose.x) ** 2 + (estimation.state.pose.y - interpolated_ground_truth.state.pose.y) ** 2)
                    self.distance_error.append(distance_error)
                    error_value_msg.data = distance_error
                    self.distance_error_pub.publish(error_value_msg)
                    
                    #ORIENTATION ERROR
                    orientation_error = numpy.abs(estimation.state.pose.phi - interpolated_ground_truth.state.pose.phi)
                    
                    if (orientation_error > numpy.pi):
                        orientation_error = 2.0 * numpy.pi - orientation_error ;    
                        
                    if (orientation_error > numpy.pi):
                        raise Exception("estimation state phi %lf, ground truth phi %lf", estimation.state.pose.phi, interpolated_ground_truth.state.pose.phi);                
                        
                    self.orientation_errors.append(orientation_error)
                    error_value_msg.data = orientation_error
                    self.orientation_error_pub.publish(orientation_error)
                    
                    #LINEAR VELOCITY ERROR
                    dlinear_error = estimation.state.twist.linear - interpolated_ground_truth.state.twist.linear
                    self.d_linear_errors.append(dlinear_error)
                    error_value_msg.data = dlinear_error;
                    self.linear_error_pub.publish(error_value_msg)
                    
                    #LINEAR ANGULAR ERROR
                    dangular_error = estimation.state.twist.angular - interpolated_ground_truth.state.twist.angular
                    self.d_angular_errors.append(dangular_error)
                    error_value_msg.data = dangular_error;
                    self.angular_error_pub.publish(error_value_msg)
                    
                    #rospy.loginfo(estimation)
                    #rospy.loginfo(interpolated_ground_truth)
        
                    #pop older ground truths
                    #rospy.loginfo("removing gorund turth from len(%lf)",len(self.ground_truth_buffer))
                    self.ground_truth_buffer = self.ground_truth_buffer[selected_ground_truth_index:-1];
                    #rospy.loginfo("to gorund turth from len(%lf)",len(self.ground_truth_buffer))
                    
                    #pop older estimations
                    self.estimation_buffer = self.estimation_buffer = self.estimation_buffer[1:-1]
                    break;
        
        else:
            rospy.logwarn("No ground truth data")            

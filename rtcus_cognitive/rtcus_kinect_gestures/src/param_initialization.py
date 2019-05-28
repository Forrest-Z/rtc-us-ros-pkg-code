import roslib
roslib.load_manifest('rtcus_kinect_gestures')
import rospy
import rosparam

class GestureParams():
    
    def __init__(self,fixed_config_files=None):
        self.fixed_config_files=fixed_config_files
        self.reload_root_config_files()
        self.refresh()
        
        self.clear_parameteres=["name","num_demonstrations","time_per_demonstration","sampling_frequency","frames","seconds_to_start_recording","wait_seconds","markers/markers_plot","markers/markers_max","markers/markers_size"]
        
    def reload_root_config_files(self):
        for config_file,name in self.fixed_config_files:
            self.load_config_file(config_file)
        
    def clear_parameters(self):
        for param in self.clear_parameteres:
            if rospy.has_param(param):
                rospy.delete_param(param)
        
    def refresh(self):
        # name of the gesture to be recorded (only needed when teaching)
        self.GESTURE_NAME = rospy.get_param('name',None)
        if self.GESTURE_NAME!=None:
            self.OUTPUT_FILE_NAME = self.GESTURE_NAME + ".yaml"
            
        # output directory where to store the result files (teaching)
        self.OUTPUT_DIR = roslib.packages.get_pkg_dir('rtcus_kinect_gestures') + '/data/'
        
        #loading parameters main parameters
        # number of demonstrations of the gesture to be performed by the user
        self.NUM_DEMONSTRATIONS = rospy.get_param('num_demonstrations',1)
        # time of recording per demonstration, in seconds
        self.TIME_PER_DEMONSTRATION = rospy.get_param('time_per_demonstration',10.0)
        self.SAMPLING_FREQUENCY = rospy.get_param('sampling_frequency',10.0)
        self.FRAMES = rospy.get_param('frames')
        
        #loading parameters optinal parameters
        self.SECONDS_TO_START_RECORDING=rospy.get_param("seconds_to_start_recording",0.0)
        # waiting time between demonstrations, in seconds
        self.WAIT_SECONDS = rospy.get_param('wait_seconds',0.0)
        # Show markers of the gesture in real time with rviz?
        self.MARKERS_PLOT = rospy.get_param('markers/markers_plot',False)
        # Maximum number of markers to be rendered at the same time
        self.MARKERS_MAX = rospy.get_param('markers/markers_max',100)
        # Size of the markers
        self.MARKERS_SIZE = rospy.get_param('markers/markers_size',1.0)
        # do a scatter plot of the recorded points at the end of each demo?
    
    def load_config_file(self,config_file):
        #rospy.loginfo(rosparam.load_file(config_file))
        rosparam.yamlmain(["rosparam", "load",config_file])
        
    def load_config_files(self,config_files):
        for (full_config_file_path,cf) in config_files:
            #rospy.loginfo("configuration loaded")
            self.load_config_file(full_config_file_path)
            
            
    
    
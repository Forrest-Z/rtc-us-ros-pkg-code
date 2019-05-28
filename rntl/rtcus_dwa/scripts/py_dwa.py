import roslib
roslib.load_manifest("rtcus_gpu_dwa")
import rospy
from actionlib.simple_action_server import SimpleActionServer


#HACE FALTA UN COSTMAP NODE

class PyDwa():
    def __init__(self):    
        self._server_name = server_name
        self._action_spec = action_spec

        
        self._action_server = SimpleActionServer(
                self._server_name,
                self._action_spec,
                execute_cb = self.execute_cb,
                auto_start = False)
        
    def execute_cb(self, goal):
        # If the state machine is running, we should preempt it before executing it
        # it again.
        rospy.logdebug("Starting wrapped SMACH container") 
    
        # Accept goal
        #goal = self._action_server.accept_new_goal()


if __name__=="main":


    
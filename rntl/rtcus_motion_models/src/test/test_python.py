import libpython_rtcus_motion_models
from libpython_rtcus_motion_models import *
from pylab import *

initial_state=StampedPose(Time(0),"local_frame")
initial_state.get_data().orientation.w=1

sim_time=Duration(2.0)
dt=Duration(0.1)

action=Twist()
action.linear.x=1.0
action.angular.z=1.0

motion_model=DeterministicNonHolonomic2D()
traj=StampedPoseVec()
motion_model.estimateStates(initial_state,action,sim_time,dt,traj,True)
times= [traj[i].get_data().position.x for i in xrange(len(traj))]
x=[traj[i].get_data().position.x for i in xrange(len(traj))]
y=[traj[i].get_data().position.y for i in xrange(len(traj))]

plot(x,y)
axis('equal')
show()
Instructions to install ROS/Soar package (software and demos):
--------------------------------------------------------------

These instructions are valid for:
- Ubuntu 10.04 LTS, 10.10, 11.04, 11.10, 12.04 LTS and 12.10 versions (both 32 or 64 bits).
- Soar 9.3.1 version (both 32 or 64 bits). 
- ROS electric and diamond versions.


0) Install ROS-Electric full desktop version following instructions in ROS web site. Renember to execute command "chown -R user /opt/ros/electric", where "user" is the name of the current user. ROS-Diamond version is also OK but it is an older version of ROS. In this case substitute packages names "ros-electric-..." by "ros-diamond-..." in instructions below.  

1) In /opt/ros/electric/stacks directory install the rt-us-ros-pkg stack using subversion:
In a terminal type "svn co https://rtc-us-ros-pkg.svn.sourceforge.net/svnroot/rtc-us-ros-pkg rtc-us-ros-pkg". After that, a new folder/directory named "rt-us-ros-pkg" should now appear.

2) Install the erratic package: in a terminal type: "sudo apt-get install ros-electric-erratic-robot". For the demo with the Erratic robot and Hokuyo Laser Scan it is also neccesary to install the laser drivers. In this case, type in a terminal: "sudo apt-get install ros-electric-laser-drivers"

3) Copy "Range.h" file to /opt/ros/electric/stacks/erratic_robot/erratic_player/msg_gen/cpp/include/erratic_player directory. This is neccesary for compability with the Gazebo simulator demo during the compilation process. This file can be found in .../rtc-us-ros-pkg/trunk/rtcus_cognitive/ros_soar/src.

4) Go to "/opt/ros/electric/stacks/rt-us-ros-pkg/trunk/rtcus_cognitive/soar/lib" and unzip Soar library "SoarSuite-9.3.1-linux-x86_32-libraries.zip" if the PC Ubuntu version is 32 bits or “SoarSuite-9.3.1-linux-x86_64-libraries.tar.gz” if  it is a Ubuntu 64 version. Unziped files must be copied/moved to this directory (do not let unziped files inside a folder/directory). Soar libraries and software for Linux can be downloaded from http://sitemaker.umich.edu/soar/downloads. 

5) In "/opt/ros/electric/stacks/rt-us-ros-pkg/trunk/rtcus_cognitive/ros_soar" directory must be compiled the executable programms. It is convenient to delete first "CmakeFiles" directory and "CMakeCache.txt" and "cmake_install.cmake" from previous compilations. Just type "make" in the terminal. It also works "cmake ." and then "make" commands. "CMakeLists.txt" file can be edited and modified (comment or uncomment lines) in order to generate only the exacutables that are needed and thus, speed up compilation process.

6) To run Gazebo demo type "roslaunch ros_soar erratic_cognitive_gazebo.launch" in a terminal. To run the Erratic robot demo type "roslaunch ros_soar ros_soar erratic_cognitive_hokuyo.launch". Please note that in this last case, the computer should be connected to a pyshical Erratic robot and the Hokuyo URG-04LX Laser too.
(NOTE: rarely, Soar or Gazebo node does load properly in the Gazebo simulation due to bugs in the libraries. Some messages appeard reporting this issue in the terminal during the launch process. In this case, it is convenient to restart the PC and launch again the Gazebo demo)

(COMMENTS: in /opt/ros/electric/stacks/rt-us-ros-pkg/trunk/rtcus_cognitive/ros_soar/src it can be found the .cpp source files of the two ROS-Soar demos: "soar_test_erratic_hokuyo.cpp" and "soar_test_gazebo.cpp". Both files are almost equal, but the first one has some parametres tuned for the Erratic robot and the second for the Gazebo simulation. The launch files associated with these two .cpp files in the "launch" directory are "erratic_cognitive_gazebo.launch" and "erratic_cognitive_hokuyo.launch", respectively. All these files can be modified, recompiled (see point 4 above) and used as a template in other applications.)


- ROS fuerte and groovy versions.

1) In both ROS fuerte and groovy versions Gazebo demos can crash due to problems with the Gazebo libraries. ROS-Soar demo can be run with Gazebo 1.2 version but could have problems with 1.3 version. Please edit stacks/simulator_gazebo/trunk/gazebo/scripts/setup.sh file and add export OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.7.4 if it is a Ubuntu 64 bits version.

2) In ROS groovy version it is convenient to create a new catkin project and then move and adapt ros packages/stacks. Using 'cmake .' and 'make' it is possible to run the erratic robot demo.   


May 2017
Daniel Cagigas-Muñiz
(dcagigas@us.es)





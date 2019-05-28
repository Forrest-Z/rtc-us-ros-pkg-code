#!/usr/bin/env python  
import roslib
roslib.load_manifest('rtcus_kinect_gestures')
import rospy
import math
import tf
import datetime
import time

OUTPUT_DIR = "/home/tomas/points/"
OUTPUT_FILE_NAME = "hand_elbow_dataset_"
NUM_DEMONSTRATIONS = 4
TIME_PER_DEMONSTRATION = 5

if __name__ == '__main__':
    rospy.init_node('register')

    current_dem = -1
    new_dem = True
    last_sentence = True

    pointsFile = None

    xElements = []
    yElements = []
    zElements = []

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
	while current_dem < NUM_DEMONSTRATIONS:
		if new_dem:
			current_dem += 1
			if pointsFile != None:
				pointsFile.close()
			if current_dem < NUM_DEMONSTRATIONS:
				rospy.loginfo('Waiting up to 15 seconds for psi pose.\n')
				listener.waitForTransform('/right_elbow', '/right_hand', rospy.Time(0), rospy.Time(15))
				output_file = OUTPUT_DIR + OUTPUT_FILE_NAME + str(current_dem) + ".txt"
				pointsFile = file( output_file, "w")
				seconds = 5
				while seconds > 0:
					rospy.loginfo("New demonstration register in " + str(seconds) + " secs.")
					time.sleep(1)
					seconds -= 1
				new_dem = False
				date = datetime.datetime.now()
				rospy.loginfo("Start")
				time.sleep(0.5)
				init_time = time.time()
			else:
				break
        	try:
            		(trans,rot) = listener.lookupTransform('/right_elbow', '/right_hand', rospy.Time(0))
        	except (tf.LookupException, tf.ConnectivityException):
            		continue

		#s = 'x: ' + str(trans[0]) + '. y: ' + str(trans[1]) + '. z: ' + str(trans[2]) + '.\n'
		#rospy.loginfo(s)

		xElements.append(trans[0])
		yElements.append(trans[1])
		zElements.append(trans[2])

		curr_time = time.time()
		if curr_time - init_time > TIME_PER_DEMONSTRATION:
			rospy.loginfo('Demonstration no. ' + str(current_dem) + ' completed.')
			new_dem = True
		    	pointsFile.write("Points captured at " + date.strftime("%Y-%m-%d %H:%M") + "\n")
		    	pointsFile.write("-" * 100 + "\n")
		  	pointsFile.write("-" * 10 + " x coordinates " + "-" * 10 + "\n")
		    	for xpt in xElements:
				pointsFile.write(str(xpt) + "\n")
		    	pointsFile.write("\n\n" + "-" * 10 + " y coordinates " + "-" * 10 + "\n")
		    	for ypt in yElements:
				pointsFile.write(str(ypt) + "\n")
		    	pointsFile.write("\n\n" + "-" * 10 + " z coordinates " + "-" * 10 + "\n")
		    	for zpt in zElements:
				pointsFile.write(str(zpt) + "\n")
        	rate.sleep()
	if last_sentence:
		rospy.loginfo('All (' + str(NUM_DEMONSTRATIONS) + ') demonstrations completed. You can now turn this node off')
		last_sentence = False

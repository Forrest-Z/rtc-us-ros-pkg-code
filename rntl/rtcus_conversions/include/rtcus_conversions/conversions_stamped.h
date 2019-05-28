/*
 * conversions_stamped.h
 *
 *  Created on: Jan 10, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CONVERSIONS_STAMPED_H_
#define CONVERSIONS_STAMPED_H_

#include<rtcus_nav_msgs/DynamicState2D.h>
#include<rtcus_nav_msgs/DynamicState2DStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_nav_msgs/StampedTwist2D.h>

namespace rtcus_conversions {

template<typename SrcType>
class StampedConversion;

//--------------------------------------------------------------------------------
template<>
class StampedConversion<rtcus_nav_msgs::Twist2D> {
public:
	typedef typename rtcus_nav_msgs::StampedTwist2D StampedType;
	static void convert(
			const rtcus_stamp::Stamped<rtcus_nav_msgs::Twist2D>& src,
			StampedType& dst)

			{
		dst.twist = *src;
		dst.header.frame_id = src.getFrameId();
		dst.header.stamp = src.getStamp();
	}

	static rtcus_nav_msgs::Twist2D remove_stamp(const StampedType& stamped) {
		return stamped.twist;
	}
};

//------------------------------------------------------------------------------------
template<>
class StampedConversion<rtcus_nav_msgs::DynamicState2D> {
public:
	typedef typename rtcus_nav_msgs::DynamicState2DStamped StampedType;
	static void convert(
			const rtcus_stamp::Stamped<rtcus_nav_msgs::DynamicState2D>& src,
			StampedType& dst)

			{
		dst.state = *src;
		dst.header.frame_id = src.getFrameId();
		dst.header.stamp = src.getStamp();
	}
	static rtcus_nav_msgs::DynamicState2D remove_stamp(
			const StampedType& stamped) {
		return stamped.state;
	}
};

//------------------------------------------------------------------------------------
template<>
class StampedConversion<geometry_msgs::Pose> {
public:
	typedef typename geometry_msgs::PoseStamped StampedType;
	static void convert(const rtcus_stamp::Stamped<geometry_msgs::Pose>& src,
			StampedType& dst)

			{
		dst.pose = *src;
		dst.header.frame_id = src.getFrameId();
		dst.header.stamp = src.getStamp();
	}
	static geometry_msgs::Pose remove_stamp(const StampedType& stamped) {
		return stamped.pose;
	}
};
}

#endif /* CONVERSIONS_STAMPED_H_ */

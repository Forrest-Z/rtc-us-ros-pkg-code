/*
 * python_binding.cpp
 *
 *  Created on: Jun 1, 2012
 *      Author: geus
 */

#include <boost/python.hpp>
#include <list>
#include <rtcus_motion_models/motion_models/non_holonomic_2d.h>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/container_utils.hpp>

using namespace boost::python;
using namespace rtcus_motion_models;
using namespace std;
//questions about this: http://answers.ros.org/question/35473/exposing-c-api-through-boostpython-and-common_msgs/

BOOST_PYTHON_MODULE(libpython_rtcus_motion_models)
{
  using namespace rtcus_motion_models;
  class_<rtcus_motion_models::DeterministicNonHolonomic2D<Pose, Twist> >("DeterministicNonHolonomic2D").def(
      "predictState", &DeterministicNonHolonomic2D<Pose, Twist>::predictState).def(
      "predictStates", &DeterministicNonHolonomic2D<Pose, Twist>::predictStates<StampedData<Pose> >).def(
          "predictStatesFromActions", &DeterministicNonHolonomic2D<Pose, Twist>::predictStatesFromActions<StampedData<Pose>,StampedData<Twist> >);

  //http://stackoverflow.com/questions/1571054/boost-python-how-to-return-by-reference
  class_<StampedData<Pose> >("StampedPose", init<ros::Time, string>()).def(
      "get_data", &StampedData<Pose>::getData, return_value_policy<reference_existing_object>()).def(
      "get_stamp", &StampedData<Pose>::getStamp);

  boost::python::class_<vector<StampedData<Pose> > >("StampedPoseVec").def(
      vector_indexing_suite<vector<StampedData<Pose> > >());

  boost::python::class_<vector<StampedData<Twist> > >("StampedActionVec").def(
      vector_indexing_suite<vector<StampedData<Twist> > >());

  class_<Point>("Point").def_readwrite("x", &Point::x).def_readwrite("y", &Point::y).def_readwrite("z", &Point::z);

  class_<Vector3>("Vector3").def_readwrite("x", &Vector3::x).def_readwrite("y", &Vector3::y).def_readwrite("z",
                                                                                                           &Vector3::z);

  class_<Quaternion>("Quaternion").def_readwrite("x", &Quaternion::x).def_readwrite("y", &Quaternion::y).def_readwrite(
      "z", &Quaternion::z).def_readwrite("w", &Quaternion::w);

  class_<Pose>("Pose").def_readwrite("position", &Pose::position).def_readwrite("orientation", &Pose::orientation);

  class_<Twist>("Twist").def_readwrite("linear", &Twist::linear).def_readwrite("angular", &Twist::angular);

  class_<ros::Duration>("Duration", init<float>());
  class_<ros::Time>("Time", init<float>());
}


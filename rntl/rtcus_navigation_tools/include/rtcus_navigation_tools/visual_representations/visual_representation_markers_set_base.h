/*
 * visual_representation_markers_set_base.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef VISUAL_REPRESENTATION_MARKERS_SET_BASE_H_
#define VISUAL_REPRESENTATION_MARKERS_SET_BASE_H_

#include <rtcus_navigation_tools/visual_representations/visual_representation.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace rtcus_navigation_tools
{
using namespace visualization_msgs;
using namespace boost;

class VisualizationMarkerSet
{
public:
  VisualizationMarkerSet()
  {
    last_markers_count_ = 0;
  }
  std::list<Marker> markers_;
  std::string ns;
  unsigned long last_markers_count_;
  int representation_type;
};

/*
 * \brief this class has been designed to print markers in an easy way forgeting the publishers, memory handling and
 *  removing old markers.
 *  \remarks the markers are removed always the protected methods: publish_all or publisn_and_clean_collision_marker
 * */
class VisualRepresetationMarkersBase : public VisualRepresetationBase
{
private:
  std::list<VisualizationMarkerSet> marker_buffers_by_namespace;
  shared_ptr<MarkerArray> markers_msg_;

public:
  VisualRepresetationMarkersBase(const std::string& representation_frame_name, const std::string& topic_name,
                                 ros::NodeHandle& n) :
      VisualRepresetationBase(representation_frame_name, n, n.advertise<MarkerArray>(topic_name, 0)), marker_buffers_by_namespace()
  {
    markers_msg_ = make_shared<MarkerArray>();
  }

  virtual ~VisualRepresetationMarkersBase()
  {
  }

protected:

  /*
   * \brief this method stores and orders in different namespace any kind of markers.
   * The reference frame for all the markers is given by this class.
   * */
  virtual void push_markers(const std::list<Marker>& representation_markers)
  {
    BOOST_FOREACH (const Marker & m, representation_markers)
    {
      //find the MarkerSet associated to this namespace
      std::list<rtcus_navigation_tools::VisualizationMarkerSet>::iterator it = std::find_if(
          this->marker_buffers_by_namespace.begin(), this->marker_buffers_by_namespace.end(),
          boost::bind(&VisualizationMarkerSet::ns, _1) == m.ns);

      VisualizationMarkerSet * current;
      if (it == this->marker_buffers_by_namespace.end())
      {
        this->marker_buffers_by_namespace.push_back(VisualizationMarkerSet());
        current = &this->marker_buffers_by_namespace.back();
        current->ns = m.ns;
        current->representation_type = Marker::SPHERE;

      }
      else
      {
        current = &(*it);
      }

      Marker mm(m);
      mm.id = current->markers_.size();
      mm.header.frame_id = this->representation_frame_name_;
      mm.header.stamp = ros::Time();
      mm.lifetime = ros::Duration(10.0);
      current->markers_.push_back(mm);
    }
  }

//TODO: make this private soon as possible. The two previous existings methods are enough for handling the problem
  virtual void publisn_and_clean_collision_marker(std::list<Marker>& markers, const char* namespace_name,
                                                  ros::Publisher& publisher, unsigned long& last_collision_count,
                                                  const std::string& frame_id, int MarkerType) const
  {

    ROS_DEBUG(
        "reserved markers size %ld, dimension %ld, required size %ld ", markers_msg_->markers.capacity(), markers_msg_->markers.size(), markers.size());
    markers_msg_->markers.resize(0);
    if (markers_msg_->markers.size() < markers.size())
    {
      ROS_DEBUG("resizing...");
      markers_msg_->markers.reserve(markers.size());
      ROS_DEBUG("resizing DONE...");
    }
    ROS_DEBUG("resulting size %ld, capacity %ld", markers_msg_->markers.size(), markers_msg_->markers.capacity());

// std::copy(marker_buffer.begin(), marker_buffer.end(), markers.markers.begin());
//MODIFY MARKERS

    std::list<Marker>::iterator it = markers.begin();
    for (unsigned long i = 0; i < std::min(last_collision_count, (unsigned long)markers.size()); i++, it++)
    {
      ROS_DEBUG("modifiing... %ld", i);
      Marker& current = *it;
      current.action = Marker::MODIFY;
      markers_msg_->markers.push_back(current);
    }

    ROS_DEBUG(
        "resulting size %ld, capacity %ld", (unsigned long)markers_msg_->markers.size(), (unsigned long) markers_msg_->markers.capacity());

    //NEW MARKERS
    if (last_collision_count < markers.size())
    {

      for (unsigned long i = last_collision_count; i < markers.size(); i++, it++)
      {
        ROS_DEBUG("new markers... %ld", i);
        Marker& current = *it;
        current.action = Marker::ADD;
        markers_msg_->markers.push_back(current);
      }
      ROS_DEBUG(
          "resulting size %ld, capacity %ld", (unsigned long) markers_msg_->markers.size(), (unsigned long) markers_msg_->markers.capacity());
    }
    //REMOVE MARKERS
    else
    {
      for (unsigned long i = markers.size(); i < last_collision_count; i++, it++)
      {
        ROS_DEBUG("removing markers.. %ld", i);
        Marker current;
        current.ns = namespace_name;
        current.id = i;

        current.header.frame_id = frame_id;
        current.header.stamp = ros::Time();
        current.type = MarkerType;
        current.action = Marker::DELETE;
        markers_msg_->markers.push_back(current);
      }
      ROS_DEBUG(
          "resulting size %ld, capacity %ld", (unsigned long) markers_msg_->markers.size(), (unsigned long)markers_msg_->markers.capacity());
    }
    ROS_DEBUG("publishing..");
    last_collision_count = std::max(last_collision_count, (unsigned long)markers.size());
    publisher.publish(markers_msg_);
    markers.clear();
  }

  virtual void publish_all_markers()
  {
    BOOST_FOREACH(VisualizationMarkerSet & current, this->marker_buffers_by_namespace)
    {

      publisn_and_clean_collision_marker(current.markers_, current.ns.c_str(), this->representation_publisher_,
                                         current.last_markers_count_, this->representation_frame_name_,
                                         current.representation_type);

    }
  }
};
}

#endif /* VISUAL_REPRESENTATION_MARKERS_SET_BASE_H_ */

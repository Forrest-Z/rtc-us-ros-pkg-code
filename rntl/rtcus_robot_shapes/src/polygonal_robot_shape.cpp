/*
 * polygonal_robot_shape.cpp
 *
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_assert/rtcus_assert.h>
#include <boost/foreach.hpp>
#include <limits>
#include <rtcus_robot_shapes/polygonal_robot.h>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Point32.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>
#include <list>

namespace rtcus_robot_shapes
{

using namespace std;

PolygonalRobot::PolygonalRobot()
{
}

PolygonalRobot::PolygonalRobot(const PolygonalRobot& copy) :
    ConfigurableShape<PolygonalRobotShapeConfig>::ConfigurableShape(copy)
{

}

PolygonalRobot::PolygonalRobot(const PolygonalRobotShapeConfig& config)
{

}

PolygonalRobot::~PolygonalRobot()
{
}

PolygonalRobot& PolygonalRobot::operator =(const PolygonalRobot& config)
{

  if (this->footprint != config.footprint)
  {
    *((PolygonalRobotShapeConfig*)this) = config;
    this->setConfig(config);
    ROS_INFO_STREAM("AFTER ASSIGN prev " << *this);
  }
  return *this;
}

void PolygonalRobot::setPoints(const vector<pcl::PointXY>& points)
{
  RTCUS_ASSERT(points.size() >= 3);

  this->maxium_radius_ = numeric_limits<double>::min();
  this->points_.clear();
  this->triangle_buffer_.clear();

  BOOST_FOREACH(const pcl::PointXY& p, points)
  {
    this->points_.push_back(p);
    double distance = sqrt(p.x * p.x + p.y * p.y);
    if (this->maxium_radius_ < distance)
      this->maxium_radius_ = distance;
  }
}

void PolygonalRobot::clear()
{
  this->footprint = "[]";
  this->points_.clear();
  this->triangle_buffer_.clear();
}

void PolygonalRobot::setRadius(double robot_radious)
{
  //ROS_WARN(
  //"Polygonal Robot Shape. The method setRadius cannot be directly used. Use setPoints instead. Shape remains in the previous state.");
  RTCUS_ASSERT_MSG(false, "NotImplemented");
}

/**
 * \remarks Based on the BSD ROS Code  for Costmap2D. developed by Eitan Marder-Eppstein.
 * */
const PolygonalRobotShapeConfig& PolygonalRobot::setConfig(const PolygonalRobotShapeConfig& config)
{

  //check and configure a new robot footprint
  //accepts a list of points formatted [[x1, y1],[x2,y2],....[xn,yn]]
  string footprint_string = config.footprint;
  boost::erase_all(footprint_string, " ");
  boost::char_separator<char> sep("[]");
  boost::tokenizer<boost::char_separator<char> > tokens(footprint_string, sep);
  vector<string> points(tokens.begin(), tokens.end());

  //parse the input string into points
  vector<pcl::PointXY> footprint_spec;
  bool valid_foot = true;

  if (points.size() >= 3)
  {
    BOOST_FOREACH(string t, tokens)
    {
      //ROS_INFO("token %s", t.c_str());
      if (t != ",")
      {
        boost::erase_all(t, " ");
        boost::char_separator<char> pt_sep(",");
        boost::tokenizer<boost::char_separator<char> > pt_tokens(t, pt_sep);
        std::vector<string> point(pt_tokens.begin(), pt_tokens.end());

        if (point.size() != 2)
        {
          ROS_WARN("Each point must have exactly 2 coordinates");
          valid_foot = false;
          break;
        }

        vector<double> tmp_pt;
        BOOST_FOREACH(string p, point)
        {
          istringstream iss(p);
          double temp;
          if (iss >> temp)
          {
            tmp_pt.push_back(temp);
          }
          else
          {
            ROS_WARN("Each coordinate must convert to a double.");
            valid_foot = false;
            break;
          }
        }
        if (!valid_foot)
          break;

        pcl::PointXY pt;
        pt.x = tmp_pt[0];
        pt.y = tmp_pt[1];

        footprint_spec.push_back(pt);
      }
    }

    if (valid_foot)
    {
      ROS_WARN("shape object. Valid robot shape: %s", config.footprint.c_str());
      *((PolygonalRobotShapeConfig*)this) = config;
      this->points_.clear();
      this->setPoints(footprint_spec);
      ROS_WARN("Computed shape size: %ld", footprint_spec.size());
      ROS_WARN("shape object. Computed radius: %lf", this->getRadius());
      ROS_WARN("OK");
      //ROS_INFO("radius-> %lf",this->getRadius());
      //ROS_INFO_STREAM("Current shape: " <<*this);
    }
    else
    {
      //at least a closed triangle
      ROS_WARN("Receiving robot shape: %s, +4?", config.footprint.c_str());
      ROS_WARN("INVALID POLYGONAL SHAPE. IGNORING");
    }
  }
  else
    ROS_WARN("Robot Configuration didn't worked");

  return *this;
}

struct FaceInfo2
{
  FaceInfo2()
  {
  }
  int nesting_level;

  bool in_domain()
  {
    return nesting_level % 2 == 1;
  }
};

/*http://www.cgal.org/Manual/3.3/doc_html/cgal_manual/Triangulation_2/Chapter_main.html#Section_25.7*/
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> VertexType;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K> FaceTypeWithInfo;
typedef CGAL::Constrained_triangulation_face_base_2<K, FaceTypeWithInfo> FaceType;
typedef CGAL::Triangulation_data_structure_2<VertexType, FaceType> TDS;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
typedef CDT::Point Point;
typedef CGAL::Polygon_2<K> Polygon_2;

void mark_domains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border)
{
  if (start->info().nesting_level != -1)
  {
    return;
  }
  std::list<CDT::Face_handle> queue;
  queue.push_back(start);

  while (!queue.empty())
  {
    CDT::Face_handle fh = queue.front();
    queue.pop_front();
    if (fh->info().nesting_level == -1)
    {
      fh->info().nesting_level = index;
      for (int i = 0; i < 3; i++)
      {
        CDT::Edge e(fh, i);
        CDT::Face_handle n = fh->neighbor(i);
        if (n->info().nesting_level == -1)
        {
          if (ct.is_constrained(e))
            border.push_back(e);
          else
            queue.push_back(n);
        }
      }
    }
  }
}

void mark_domains(CDT& cdt)
{
  for (CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it)
  {
    it->info().nesting_level = -1;
  }

  int index = 0;
  std::list<CDT::Edge> border;
  mark_domains(cdt, cdt.infinite_face(), index++, border);
  while (!border.empty())
  {
    CDT::Edge e = border.front();
    border.pop_front();
    CDT::Face_handle n = e.first->neighbor(e.second);
    if (n->info().nesting_level == -1)
    {
      mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
    }
  }
}

void PolygonalRobot::getTriangularization(std::vector<pcl::PointXY>& triangle_buffer)
{
  RTCUS_ASSERT(this->points_.size() >= 3);
  if (this->triangle_buffer_.size() == 0) //check if cached
  {
    CDT cdt;
    for (unsigned int i = 0; i < this->points_.size() - 1; i++)
    {
      Point p1(this->points_[i].x, this->points_[i].y);
      Point p2(this->points_[i + 1].x, this->points_[i + 1].y);
      cdt.insert_constraint(p1, p2);
      ROS_INFO_STREAM(p1<< "-"<< p2);
    }

    Point p1(this->points_.back().x, this->points_.back().y);
    Point p2(this->points_.front().x, this->points_.front().y);
    cdt.insert_constraint(p1, p2);
    ROS_INFO_STREAM(p1<< "-"<< p2);

    int count = 0;
    for (CDT::Finite_edges_iterator eit = cdt.finite_edges_begin(); eit != cdt.finite_edges_end(); ++eit)
      if (cdt.is_constrained(*eit))
        ++count;
    std::cout << "The number of resulting constrained edges is  ";
    std::cout << count << std::endl;

    //select internal faces
    mark_domains(cdt);

    for (CDT::Finite_faces_iterator it = cdt.finite_faces_begin(); it != cdt.finite_faces_end(); ++it)
    {
      ROS_INFO("new triangle");
      if (it->info().in_domain())
      {
        pcl::PointXY p;
        p.x = it->vertex(0)->point().x();
        p.y = it->vertex(0)->point().y();
        this->triangle_buffer_.push_back(p);
        ROS_INFO_STREAM(p.x<< ","<< p.y);

        p.x = it->vertex(1)->point().x();
        p.y = it->vertex(1)->point().y();
        this->triangle_buffer_.push_back(p);
        ROS_INFO_STREAM(p.x<< ","<< p.y);

        p.x = it->vertex(2)->point().x();
        p.y = it->vertex(2)->point().y();
        this->triangle_buffer_.push_back(p);
        ROS_INFO_STREAM(p.x<< ","<< p.y);
      }
      else
      {
        ROS_INFO("Discarding triangle because it outside.");
      }
    }
  }
  triangle_buffer = this->triangle_buffer_;
}
;

void PolygonalRobot::getROSPolygon(geometry_msgs::Polygon& polygon) const
{
  polygon.points.clear();
  BOOST_FOREACH(const pcl::PointXY& p , this->points_)
  {
    geometry_msgs::Point32 pros;
    pros.x = p.x;
    pros.y = p.y;
    pros.z = 0;
    polygon.points.push_back(pros);
  }
}

const vector<pcl::PointXY>& PolygonalRobot::getPoints() const
{
  return points_;
}

ostream& operator <<(ostream &out, const PolygonalRobot &polygonal_shape)
{

  out << "[";
  BOOST_FOREACH(const pcl::PointXY& p , polygonal_shape.getPoints())
  {
    out << "[ " << p.x << "," << "," << p.y << "]";
  }
  out << "]-> footprint: " << polygonal_shape.footprint;
  return out;
}

}


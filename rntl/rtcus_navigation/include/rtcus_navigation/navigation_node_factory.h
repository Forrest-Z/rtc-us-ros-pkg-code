/*
 * navigation_node_factory.h
 *
 *  Created on: Nov 20, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NAVIGATION_NODE_FACTORY_H_
#define NAVIGATION_NODE_FACTORY_H_

namespace rtcus_navigation
{
#define NAVIGATION_TYPES(NavPlanner) \
typedef typename NavPlanner::TNavigationNodePtr TNavigationNodePtr;\
typedef typename NavPlanner::TNavigationNode TNavigationNode;\
typedef typename NavPlanner::TNavigationPlanner TNavigationPlanner

template<class NavigationPlanner>
  class NavigationNodeFactory
  {
  private:
    NavigationNodeFactory()
    {

    }
  public:
    NAVIGATION_TYPES(NavigationPlanner);

    //default construction
    static TNavigationNodePtr create_navigation_node();

    //specific construction
    static TNavigationNodePtr create_navigation_node(const std::string& type);

  };
}
#endif /* NAVIGATION_NODE_FACTORY_H_ */

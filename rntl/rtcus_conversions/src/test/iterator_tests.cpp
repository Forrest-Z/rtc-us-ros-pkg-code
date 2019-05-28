/*
 * iterator_tests.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: root
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rtcus_conversions/conversions.h>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/lambda/closures.hpp>

bool Even(int& i)
{
  return i % 2 == 0;
}

bool greater(int& i)
{
  return i > 3;
}

TEST(IteratorTests,TestWhere)
{
  std::vector<int> vec;
  vec.push_back(1);
  vec.push_back(2);
  vec.push_back(6);
  vec.push_back(3);
  vec.push_back(5);
  vec.push_back(8);

  //rtcus_conversions::IterateVector<int> itv(vec);

  rtcus_conversions::Iterator<std::vector<int> > itsrc(vec);
  /*do
   {
   printf("%d\n", itsrc.getCurrent());
   } while (itsrc.moveNext());
   */
  rtcus_conversions::IterateWhere<int, std::vector<int> > itv = itsrc.where(boost::bind(Even, _1)).where(
      boost::bind(greater, _1));

  //rtcus_conversions::IterateWhere<int, std::vector<int> > itv2 = itsrc.where(boost::bind(Even, _1)).where(
  //    boost::bind(greater, _1));

  printf("hellooooooooooooooooooo\n");
  while (itv.hasNext())

  {
    printf("%d\n", itv.getCurrent());
    itv.moveNext();
  }

}

/*
 * conversions.h
 *
 *  Created on: Jun 18, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <bits/stl_iterator_base_types.h>
#include <vector>
#include <boost/function.hpp>

namespace rtcus_conversions
{
class Conversions
{
public:
  template<typename SrcType, typename DstType>
    static void convert(const SrcType& src, DstType& dst);
};

template<typename TValue, typename TContainer>
  class IEnumerator
  {

  public:
    //typedef typename container_type;
    typedef TValue value_type;
    typedef TContainer container_type;

    virtual ~IEnumerator()
    {
    }
    virtual bool hasNext();
    virtual void moveNext();
    virtual TValue& getCurrent() const;
    virtual TValue& getCurrent();

    /*
     virtual IterateWhere<TValue, TContainer> where(boost::function1<bool, TValue> cond)
     {
     return IterateWhere<TValue, TContainer>(this->getPointer(), cond);
     }*/
  };

}

#endif /* CONVERSIONS_H_ */

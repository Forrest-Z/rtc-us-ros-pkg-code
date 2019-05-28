/*
 * Stamped.h
 *
 *  Created on: Apr 11, 2012
 *      Author: Pablo Inigo Blasco
 */

#ifndef RTCUS_STAMPED_H_
#define RTCUS_STAMPED_H_

#include <ros/time.h>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <rtcus_stamp/is_headed_msg.h>
#include <ros/message_traits.h>
#include <ros/console.h>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_stamp
{

//stamped classes capacities
//explicit casting to T&
//operators
//examples of how to pass as function argument of T type
template<typename TTime = ros::Time>
  class TimeStamped
  {
  public:
    virtual TTime getStamp() const=0;
    virtual void setStamp(const TTime value)=0;
    virtual ~TimeStamped()
    {
    }
  };

class ReferenceFrameStamped
{
public:
  virtual std::string getFrameId() const=0;
  virtual void setFrameId(const std::string& frame_id)=0;
  virtual ~ReferenceFrameStamped()
  {
  }
};

enum TAllocationType
{
  StaticAllocation, SmartPtr
};

template<typename T, typename TTime, typename hasHeader = ros::message_traits::FalseType>
  class StampInfoContainer : public TimeStamped<TTime>, public ReferenceFrameStamped
  {
  protected:
    TTime stamp_;
    std::string frame_id_;

  public:
    virtual ~StampInfoContainer()
    {

    }
    virtual TTime getStamp() const
    {
      return stamp_;
    }
    virtual void setStamp(const TTime value)
    {
      stamp_ = value;
    }
    virtual std::string getFrameId() const
    {
      return frame_id_;
    }
    virtual void setFrameId(const std::string& frame_id)
    {
      frame_id_ = frame_id;
    }
    virtual T& getData()=0;
    virtual const T& getConstData() const=0;
    virtual bool hasData() const=0;
  };

template<typename T, typename TTime>
  class StampInfoContainer<T, TTime, ros::message_traits::TrueType> : public TimeStamped<TTime>,
                                                                      public ReferenceFrameStamped
  {
  public:
    virtual TTime getStamp() const
    {
      return ros::message_traits::TimeStamp<T>::pointer(this->getData());
    }
    virtual void setStamp(const TTime value)
    {
      ros::message_traits::TimeStamp<T>::pointer(this->getData()) = value;
    }
    virtual std::string getFrameId() const
    {
      return ros::message_traits::FrameId<T>::pointer(this->getData());
    }
    virtual void setFrameId(const std::string& frame_id)
    {
      ros::message_traits::FrameId<T>::pointer(this->getData()) = frame_id;
    }
  };

template<typename T, typename TTime = ros::Time>
  class Stamped : public StampInfoContainer<T, TTime, ros::message_traits::HasHeader<T> >
  {
  protected:
    boost::shared_ptr<T> data_;

  public:
    Stamped()
    {

    }
    virtual ~Stamped()
    {

    }

    Stamped(boost::shared_ptr<T> data) :
        data_(data)
    {

    }

    void setAllocatedData(boost::shared_ptr<T> data)
    {
      this->data_ = data;
    }

    boost::shared_ptr<Stamped<T, TTime> > toSharedPtr()
    {
      return boost::shared_ptr<Stamped<T, TTime> >(this, boost::serialization::null_deleter());
    }

    Stamped<T, TTime>& operator=(const Stamped<T, TTime>& other)
    {
      this->assign(other);
      return *this;
    }

    virtual void assign(const Stamped<T, TTime>& other)
    {
      this->data_ = other.data_;
      this->setFrameId(other.getFrameId());
      this->setStamp(other.getStamp());
    }

    bool operator==(const Stamped<T, TTime>& other)
    {
      return this->getData() == other.getConstData() && this->getStamp() == other.getStamp();
    }

    bool operator!=(const Stamped<T, TTime>& other)
    {
      return !(*(this) == other);
    }

    void copyFromData(const T& other)
    {
      //ROS_INFO_STREAM("copy from data: " << other);
      this->getData() = other;

    }

    void copyFrom(const Stamped<T, TTime>& other)
    {
      this->copyFromData(*other);
      this->setFrameId(other.getFrameId());
      this->setStamp(other.getStamp());
    }

    void warn_integrity(std::string event)
    {
      if (this->data_.use_count() > 1)
        ROS_WARN(
            "%s STAMPED DATA: %s but the data is being used(%ld), Eventually a segmentation fault can occur", event.c_str(), typeid(T).name(), this->data_.use_count());
    }

    T& operator*()
    {
      return this->getData();
    }

    const T& operator*() const
    {
      return this->getConstData();
    }

    T* operator->()
    {
      if (!this->data_)
      {

        ROS_ERROR_STREAM("Using null shared_ptr" << typeid(T).name());
        print_trace();
      }

      return &(this->getData());

    }

    const T* operator->() const
    {
      if (!this->data_)
      {

        ROS_ERROR_STREAM("Using null shared_ptr" << typeid(T).name());
        print_trace();
      }

      return &(this->getConstData());
    }

    virtual bool hasData() const
    {
      return this->data_;
    }

    virtual T& getData()
    {
      if (!this->data_)
      {

        ROS_ERROR_STREAM("Using null shared_ptr" << typeid(T).name());
        print_trace();
      }
      return *(this->data_);
    }

    virtual const T& getConstData() const
    {
      if (!this->data_)
      {

        ROS_ERROR_STREAM("Using null shared_ptr" << typeid(T).name());
        print_trace();
      }

      return *(this->data_);
    }

  };

//They are themselves containers but eventually can reference to external data
//to achieve a better performance avoiding copies
//The main problem is that this is not recommended for local scope work since
//memmory management is required during its construction
template<class T, typename TTime = ros::Time>
  class StampedShared : public Stamped<T, TTime>
  {
  public:
    StampedShared() :
        Stamped<T, TTime>::Stamped(boost::make_shared<T>())
    {

    }

    Stamped<T, TTime>& operator=(const Stamped<T, TTime>& other)
    {
      ((Stamped<T, TTime>&)(*this)) = other;
      return *this;
    }
  };

//An assign operation always produces a local copy of the data and the stamp
template<class T, typename TTime = ros::Time>
  class StampedData : public Stamped<T, TTime>

  {
  protected:
    T static_data_;
  public:
    StampedData() :
        Stamped<T, TTime>::Stamped(boost::shared_ptr<T>(&static_data_, boost::serialization::null_deleter())), static_data_()
    {
      this->setFrameId("");
      this->setStamp(TTime(0));
      this->warn_integrity("NON PARAMETER CONSTRUCTOR");
    }
    StampedData(const T& copy) :
        Stamped<T, TTime>::Stamped(boost::shared_ptr<T>(&static_data_, boost::serialization::null_deleter())), static_data_(
            copy)
    {
      this->setFrameId("");
      this->setStamp(TTime(0));
      this->warn_integrity("DATA COPY CONSTRUCTOR");
    }

    StampedData(const StampedData<T, TTime>& copy) :
        Stamped<T, TTime>::Stamped(boost::shared_ptr<T>(&static_data_, boost::serialization::null_deleter())), static_data_(
            *copy)
    {
      this->setFrameId(copy.getFrameId());
      this->setStamp(copy.getStamp());
      this->warn_integrity("COPY CONSTRUCTOR");
    }

    StampedData(const TTime& stamp, const std::string& frame_id) :
        Stamped<T, TTime>::Stamped(boost::shared_ptr<T>(&static_data_, boost::serialization::null_deleter())), static_data_()
    {
      this->setFrameId(frame_id);
      this->setStamp(stamp);
      this->warn_integrity("PARAMETER CONSTRUCTOR");
    }

    StampedData(const T& copy, const TTime& stamp_, const std::string& frame_id_) :
        Stamped<T, TTime>::Stamped(boost::shared_ptr<T>(&static_data_, boost::serialization::null_deleter())), static_data_(
            copy)
    {
      this->setFrameId(frame_id_);
      this->setStamp(stamp_);
      this->warn_integrity("PARAMETER CONSTRUCTOR");
    }

    virtual void assign(const Stamped<T, TTime>& other)
    {
      this->copyFrom(other);
    }

    virtual T& getData()
    {
      return (static_data_);
    }

    virtual const T& getConstData() const
    {
      return (static_data_);
    }

    virtual ~StampedData()
    {
      this->warn_integrity("DESTRUCTOR");

    }
  };

/*
 template<class T, typename TTime = ros::Time>
 class StampedMsg : public T, public BaseStamped<StampedMsg<T, TTime>, T, TTime>
 {

 public:
 StampedMsg(const T& copy) :
 T(copy), BaseStamped<StampedMsg<T, TTime>, T, TTime>::BaseStamped(boost::shared_ptr<T>(this))
 {

 }
 StampedMsg() :
 BaseStamped<StampedMsg<T, TTime>, T, TTime>::BaseStamped(boost::shared_ptr<T>(this))
 {

 }

 virtual ~StampedMsg()
 {

 }

 inline virtual std::string getFrameId() const
 {
 return this->header.frame_id;
 }

 inline virtual void setFrameId(const std::string& value)
 {
 ((T*)this)->header.frame_id = value;
 }

 inline virtual TTime getStamp() const
 {
 return this->header.stamp;
 }
 inline virtual void setStamp(const TTime value)
 {
 this->header.stamp = value;
 }
 inline virtual void copyData(const T& input)
 {
 *this = input;
 }
 inline virtual T& getData()
 {
 return *this;
 }
 virtual const T& getConstData() const
 {
 return *((const T*)this);
 }
 };


 template<class T, typename TTime = ros::Time>
 class StampedMsgNoCopyBase : public BaseStamped<StampedMsgNoCopyBase<T, TTime>, T, TTime>
 {
 protected:
 T* data_;

 StampedMsgNoCopyBase(T& data) :
 data_(&data)
 {
 }

 public:

 virtual ~StampedMsgNoCopyBase()
 {
 }

 inline virtual T& getData()
 {
 return *data_;
 }
 inline virtual T& getConstData() const
 {
 return const_cast<T&>(*data_);
 }

 inline virtual std::string getFrameId() const
 {
 return data_->header.frame_id;
 }

 inline virtual TTime getStamp() const
 {
 return data_->header.stamp;
 }

 };

 */
/*
 template<class T, typename TTime = ros::Time>
 class ConstantStampedMsgNoCopy : public StampedMsgNoCopyBase<T, TTime>
 {
 public:
 ConstantStampedMsgNoCopy(T& data) :
 StampedMsgNoCopyBase<T, TTime>(data)
 {
 }

 inline virtual void setFrameId(const std::string& value)
 {
 assert(false);
 }

 inline virtual void copyData(const T& input)
 {
 assert(false);
 }
 inline virtual void setStamp(const TTime value)
 {
 assert(false);
 }

 virtual ~ConstantStampedMsgNoCopy()
 {
 }
 };
 */
/*
 template<class T, typename TTime = ros::Time>
 class StampedMsgNoCopy : public StampedMsgNoCopyBase<T, TTime>
 {
 public:
 StampedMsgNoCopy(T& data) :
 StampedMsgNoCopyBase<T, TTime>(data)
 {
 }
 virtual ~StampedMsgNoCopy()
 {
 }
 inline virtual void copyData(const T& input)
 {
 *(this->data_) = input;
 }
 inline virtual void setStamp(const TTime value)
 {
 this->data_->header.stamp = value;
 }
 inline virtual void setFrameId(const std::string& value)
 {
 this->data_->header.frame_id = value;
 }
 };

 template<class T, typename TTime = ros::Time>
 class StampedDataNoCopy : public BaseStamped<StampedDataNoCopy<T, TTime>, T, TTime>
 {

 protected:
 TTime stamp_;
 std::string frame_id_;

 public:
 StampedDataNoCopy(boost::shared_ptr<T> data, TTime stamp = TTime(0), const std::string& frame_id = "") :
 BaseStamped<StampedDataNoCopy<T, TTime>, T, TTime>::BaseStamped(data), stamp_(stamp), frame_id_(frame_id)
 {

 }

 virtual ~StampedDataNoCopy()
 {
 }

 inline virtual std::string getFrameId() const
 {
 return frame_id_;
 }

 inline virtual void setFrameId(const std::string& value)
 {
 frame_id_ = value;
 }

 inline virtual TTime getStamp() const
 {
 return stamp_;
 }

 inline virtual void setStamp(const TTime value)
 {
 stamp_ = value;
 }

 };
 */
}

#endif /* STAMPED_H_ */

/*
 * is_headed_msg.h
 *
 *  Created on: Jun 20, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef IS_HEADED_MSG_H_
#define IS_HEADED_MSG_H_
#include<ros/time.h>

/*! \brief this generic function checks at compile time if the arguments contains a stamped header
 * and returns false if not. Else returns the information in the input/ouput arguments
 * \remarks This is now deprecated. Use ros::mesagge_traits instead.
 * */
template<typename T>
  bool isHeadedRosMsg(const T& rosmsg, ros::Time& out_time, std::string& frame_id);

/*! \remarks This is now deprecated. Use ros::mesagge_traits instead.
 * */
template<typename T>
  bool setHeaderRosMsg(T& rosmsg, const ros::Time& out_time, const std::string& frame_id);

//================= IMPLEMENTATION ======================================================================
#define CREATE_MEMBER_DETECTOR(X)                                                   \
template<typename T> class Detect_##X {                                             \
    struct Fallback { int X; };                                                     \
    struct Derived : T, Fallback { };                                               \
                                                                                    \
    template<typename U, U> struct Check;                                           \
                                                                                    \
    typedef char ArrayOfOne[1];                                                     \
    typedef char ArrayOfTwo[2];                                                     \
                                                                                    \
    template<typename U> static ArrayOfOne & func(Check<int Fallback::*, &U::X> *); \
    template<typename U> static ArrayOfTwo & func(...);                             \
  public:                                                                           \
    typedef Detect_##X type;                                                        \
    enum { value = sizeof(func<Derived>(0)) == 2 };                                 \
};

CREATE_MEMBER_DETECTOR(header)

template<bool hasheader, typename T>
  struct __function_selector_is_stamped
  {
    inline static bool execute(const T& data, ros::Time& out_time, std::string& frame_id)
    {
      return false;
    }
  };

template<typename T>
  struct __function_selector_is_stamped<true, T>
  {
    inline static bool execute(const T& data, ros::Time& out_time, std::string& frame_id)
    {
      out_time = data.header.stamp;
      frame_id = data.header.frame_id;
      return true;
    }
  };

template<typename T>
  inline bool isHeadedRosMsg(const T& rosmsg, ros::Time& out_time, std::string& frame_id)
  {
    return __function_selector_is_stamped<Detect_header<T>::value, T>::execute(rosmsg, out_time, frame_id);
  }

//-------------------------------

template<bool hasheader, typename T>
  struct __function_selector_set_stamped
  {
    inline static bool execute(T& data, const ros::Time& time, const std::string& frame_id)
    {
      return false;
    }
  };

template<typename T>
  struct __function_selector_set_stamped<true, T>
  {
    inline static bool execute(T& data, const ros::Time& time, const std::string& frame_id)
    {
      data.header.stamp = time;
      data.header.frame_id = frame_id;
      return true;
    }
  };

template<typename T>
  inline bool setHeaderRosMsg(T& rosmsg, const ros::Time& time, const std::string& frame_id)
  {
    return __function_selector_set_stamped<Detect_header<T>::value, T>::execute(rosmsg, time, frame_id);
  }

#endif /* IS_HEADED_MSG_H_ */

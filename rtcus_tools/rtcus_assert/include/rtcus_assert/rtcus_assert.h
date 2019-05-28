/*
 * auxiliar_print_stack_trace.h
 *
 *  Created on: Oct 3, 2012
 *      Author: root
 */

#ifndef AUXILIAR_PRINT_STACK_TRACE_H_
#define AUXILIAR_PRINT_STACK_TRACE_H_

#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <iostream>
#include <streambuf>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include <ros/ros.h>

//http://stackoverflow.com/questions/4636456/stack-trace-for-c-using-gcc/4732119#4732119
void print_trace();
int catch_sigsegv_and_print_trace();

#define RTCUS_ASSERT_CATCH_SIGSEGV() catch_sigsegv_and_print_trace()

#define RTCUS_ASSERT_MSG(condition, ...)   \
{                                               \
if (! (condition))                              \
{                                               \
  ROS_FATAL("Application DEAD BY RTCUS ASSERTTION (in: %s: %d): %s. ", __FILE__,__LINE__,#condition); \
  ROS_LOG(::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__); \
  print_trace();                                \
  BOOST_ASSERT(condition);              \
}                                               \
}

#define RTCUS_ASSERT(condition) RTCUS_ASSERT_MSG(condition,"<no description>")

#endif /* AUXILIAR_PRINT_STACK_TRACE_H_ */

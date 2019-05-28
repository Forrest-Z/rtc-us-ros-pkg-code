/*
 * rtcus_assert.cpp
 *
 *  Created on: Oct 3, 2012
 *      Author: root
 */
#include <rtcus_assert/rtcus_assert.h>

void seg(int a)
{
  RTCUS_ASSERT_MSG(false, "Application DEAD by signal SIGSEGV.");
}

int catch_sigsegv_and_print_trace()
{
  signal(SIGSEGV, seg);
  return 0;
}

void print_trace()
{
  char pid_buf[30];
  sprintf(pid_buf, "%d", getpid());
  char name_buf[512];
  name_buf[readlink("/proc/self/exe", name_buf, 511)] = 0;
  int child_pid = fork();
  if (!child_pid)
  {

    /*    std::filebuf buffer;
     std::streambuf * old = std::cout.rdbuf(buffer.rdbuf());
     fprintf(stdout, "stack trace for %s pid=%s\n", name_buf, pid_buf);
     execlp("gdb", "gdb", "--batch", "-n", "-ex", "thread", "-ex", "bt", name_buf, pid_buf, NULL);
     ROS_FATAL_STREAM(buffer);
     */
    dup2(2, 1); // redirect output to stderr
    fprintf(stdout, "stack trace for %s pid=%s\n", name_buf, pid_buf);
    execlp("gdb", "gdb", "--batch", "-n", "-ex", "thread", "-ex", "bt", name_buf, pid_buf, NULL);

    abort(); /* If gdb failed to start */
  }
  else
  {
    waitpid(child_pid, NULL, 0);
  }
}


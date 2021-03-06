#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <sched.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <limits.h>
#include <linux/unistd.h>

#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/utsname.h>
#include <sys/mman.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "turtlesim/Kill.h"

#define NSEC_PER_SEC 1000000000
#define TEST_LOOP_NUM 10000

#define VALBUF_SIZE    16384
#define DEFAULT_PRIO    80

#define MODE_CYCLIC		0
#define MODE_CLOCK_NANOSLEEP	1
#define MODE_SYS_ITIMER		2
#define MODE_SYS_NANOSLEEP	3
#define MODE_SYS_OFFSET		2

//calculate duration
static inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2)
{
	int64_t diff;
	diff = NSEC_PER_SEC * (int64_t)((int) t1.tv_sec - (int) t2.tv_sec);
	diff += ((int) t1.tv_nsec - (int) t2.tv_nsec);
	return diff;
}
//transform time from struct timespec to ns
static inline int64_t time_trans_ns(struct timespec t)
{
	int64_t ns;
	ns = NSEC_PER_SEC * (int64_t)t.tv_sec;
	ns += t.tv_nsec;
	return ns;
}


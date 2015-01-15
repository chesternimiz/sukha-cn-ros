#include "ros_rt_test/ros_rt_test.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<turtlesim::Kill>("test");
    int count=0;
    turtlesim::Kill srv;
    srv.request.name = "11111111111111111111111111111111111111111111111111111111111111111111111111111111111111";

    struct timespec now;
    struct timespec prev;
    struct timespec interval;
    interval.tv_sec = 0;
    interval.tv_nsec = 1000000;
    int64_t diff = 0;

    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = DEFAULT_PRIO;
    if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
        printf("set scheduler failed.\n");
        sched_setscheduler(0, SCHED_OTHER, &schedp);
    }
    if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        printf("mlock failed.\n");
    }

    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGALRM);
    sigprocmask(SIG_BLOCK, &sigset, NULL);

    ROS_INFO("client sending begin ");
    FILE *fp = fopen("/home/exbot/srv_test.txt", "wt");
    if (NULL == fp) {
        printf("open file failed.\n");
    }
    else {
        while(count < TEST_LOOP_NUM && ros::ok()){
            clock_gettime(CLOCK_MONOTONIC, &prev);
            client.call(srv);
            nanosleep(&interval,0);
            clock_gettime(CLOCK_MONOTONIC, &now);
            diff = calcdiff_ns(now, prev);
            diff = diff - 1000000;
            fprintf(fp, "%lld\t", diff);
            count++;
        }
    }
    ROS_INFO("client sending finished ");
    return 0;
}


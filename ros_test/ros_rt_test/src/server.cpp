#include "ros_rt_test/ros_rt_test.h"

//define the callback function
bool back( turtlesim::Kill::Request &req, turtlesim::Kill::Response &res)
{
    //ROS_INFO("receiving...");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"server"); 
    ros::NodeHandle n;

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
  
    ros::ServiceServer service = n.advertiseService("test",back);
    ROS_INFO("ready...");

    ros::spin();
    return 0;
}


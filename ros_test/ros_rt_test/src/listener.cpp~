#include "ros_rt_test/ros_rt_test.h"

//record whole number of received messages.
static int count = 0;

//message received callback
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I have heard %s count %d",msg->data.c_str(), count);
//  msg->data.c_str()
    count++;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    //set schedule policy and priority. the processor will be real-time then.
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

    ros::Subscriber sub = n.subscribe("chatter",10000000000000,chatterCallback);

    ros::spin();
    return 0;
}


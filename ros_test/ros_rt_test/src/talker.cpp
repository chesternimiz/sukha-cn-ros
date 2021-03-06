#include "ros_rt_test/ros_rt_test.h"


int main(int argc, char **argv)
{
    //initialize node & publisher
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(1000);

    //set schedule policy and priority. the processor will be real-time then.
    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = DEFAULT_PRIO;
    if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
        printf("set scheduler failed.\n");
        sched_setscheduler(0, SCHED_OTHER, &schedp);
    }
    //memory lock
    if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        printf("mlock failed.\n");
    }

    //block signals.
    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGALRM);
    sigprocmask(SIG_BLOCK, &sigset, NULL);

    //initialize message, about 100 byte.
    std::stringstream ss;
    std_msgs::String msg; ss<<"1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111";
    msg.data = ss.str();    

    //wait for publisher initialization 
    sleep(4);

    struct timespec now;
    struct timespec prev;
    struct timespec interval;    //nano sleep interval
    interval.tv_sec = 0;
    interval.tv_nsec = 1000000;
    int64_t diff = 0;            //duration

    //open a file to record the result
    FILE *fp = fopen("/home/exbot/test_file.txt", "wt");
    if (NULL == fp) {
        printf("open file failed.\n");
    }
    else {
        int loop_i = 0;
        while (loop_i < TEST_LOOP_NUM && ros::ok()) {
        clock_gettime(CLOCK_MONOTONIC, &prev);
        chatter_pub.publish(msg);
//      ros::spinOnce();
//      loop_rate.sleep();
//      nanosleep(&interval,0);
        clock_gettime(CLOCK_MONOTONIC, &now);
        diff = calcdiff_ns(now, prev);
//      diff = diff - 1000000;
        fprintf(fp, "%lld\t", diff);
        loop_i++;
        ros::spinOnce();
        loop_rate.sleep();
        }
        fclose(fp);
    }
}

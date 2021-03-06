#ifndef OROCOS_HELLO_WORLD_COMPONENT_HPP
#define OROCOS_HELLO_WORLD_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/main.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <std_msgs/String.h>


class hello_world : public RTT::TaskContext{
public:
        hello_world(std::string const& name);
        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        void cleanupHook();
private:
        RTT::OutputPort<std_msgs::String> outPort;
        RTT::InputPort<std_msgs::String> inPort;
};
#endif
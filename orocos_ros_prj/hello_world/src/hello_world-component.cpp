#include "hello_world-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

hello_world::hello_world(std::string const& name) : TaskContext(name)
{
        //set the priority and period
        this->setActivity( new RTT::Activity(0, 0.001));

        //add output port to hello_world component
        this->ports()->addPort("out_port", outPort);
        outPort.createStream(rtt_roscomm::topic("/chatter_write"));

        //add input port to component
        this->ports()->addPort("in_port", inPort);
        inPort.createStream(rtt_roscomm::topic("/chatter"));
}

bool hello_world::configureHook()
{
        std::cout << "hello_world configured !" <<std::endl;
        return true;
}

bool hello_world::startHook()
{
        std::cout << "Hello_world started !" <<std::endl;
        return true;
}

void hello_world::updateHook()
{
        //say hello world to ros node every period.
        std_msgs::String write_val;
        write_val.data = "hello World, orocos.";
        outPort.write( write_val );

        //listen to ros node every period.
        std_msgs::String read_val;
        if (inPort.read(read_val) == RTT::NewData) {
                std::cout << read_val.data <<std::endl;
        }
}

void hello_world::stopHook() 
{
        std::cout << "hello_world executes stopping !" <<std::endl;
}

void hello_world::cleanupHook() 
{
        std::cout << "hello_world cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Hello_world)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(hello_world)

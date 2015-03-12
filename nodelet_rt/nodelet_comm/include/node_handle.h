/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RT_NODE_HANDLE_H
#define RT_NODE_HANDLE_H

#include "ros/ros.h"

namespace nodelet_rt
{

  /**
   * \brief roscpp's interface for creating subscribers, publishers, etc.
   *
   * This class is used for writing nodes.  It provides a RAII interface
   * to this process' node, in that when the first NodeHandle is
   * created, it instantiates everything necessary for this node, and
   * when the last NodeHandle goes out of scope it shuts down the node.
   *
   * NodeHandle uses reference counting internally, and copying a
   * NodeHandle is very lightweight.
   *
   * You must call one of the ros::init functions prior to instantiating
   * this class.
   *
   * The most widely used methods are:
   *   - Setup:
   *    - ros::init()
   *   - Publish / subscribe messaging:
   *    - advertise()
   *    - subscribe()
   *   - RPC services:
   *    - advertiseService()
   *    - serviceClient()
   *    - ros::service::call()
   *   - Parameters:
   *    - getParam()
   *    - setParam()
   */
  class ROSCPP_DECL NodeHandleRT : public ros::NodeHandle
  {
  public:
    /**
     * \brief Constructor
     *
     * When a NodeHandle is constructed, it checks to see if the global
     * node state has already been started.  If so, it increments a
     * global reference count.  If not, it starts the node with
     * ros::start() and sets the reference count to 1.
     *
     * \param ns Namespace for this NodeHandle.  This acts in addition to any namespace assigned to this ROS node.
     *           eg. If the node's namespace is "/a" and the namespace passed in here is "b", all 
     *           topics/services/parameters will be prefixed with "/a/b/"
     * \param remappings Remappings for this NodeHandle.
     * \throws InvalidNameException if the namespace is not a valid graph resource name
     */
    NodeHandleRT(const std::string& ns = std::string(), const M_string& remappings = M_string()) : NodeHandle();

    /**
     * \brief Destructor
     *
     * When a NodeHandle is destroyed, it decrements a global reference
     * count by 1, and if the reference count is now 0, shuts down the
     * node.
     */
    ~NodeHandle();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Versions of advertise()
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Advertise a topic, simple version
     *
     * This call connects to the master to publicize that the node will be
     * publishing messages on the given topic.  This method returns a Publisher that allows you to
     * publish a message on this topic.
     *
     * This version of advertise is a templated convenience function, and can be used like so
     *
     *   ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);
     *
     * \param topic Topic to advertise on
     *
     * \param queue_size Maximum number of outgoing messages to be
     * queued for delivery to subscribers
     *
     * \param latch (optional) If true, the last message published on
     * this topic will be saved and sent to new subscribers when they
     * connect
     *
     * \return On success, a Publisher that, when it goes out of scope,
     * will automatically release a reference on this advertisement.  On
     * failure, an empty Publisher.
     *
     * \throws InvalidNameException If the topic name begins with a
     * tilde, or is an otherwise invalid graph resource name, or is an
     * otherwise invalid graph resource name
     */
    template <class M>
    Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
    {
      AdvertiseOptions ops;
      ops.template init<M>(topic, queue_size);
      ops.latch = latch;
      return advertise(ops);
    }

  /**
   * \brief Advertise a topic, with full range of AdvertiseOptions
   *
   * This call connects to the master to publicize that the node will be
   * publishing messages on the given topic.  This method returns a Publisher that allows you to
   * publish a message on this topic.
   *
   * This is an advanced version advertise() that exposes all options (through the AdvertiseOptions structure)
   *
   * \param ops Advertise options to use
   * \return On success, a Publisher that, when it goes out of scope, will automatically release a reference
   * on this advertisement.  On failure, an empty Publisher which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *
   * \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   */
  Publisher advertise(AdvertiseOptions& ops);


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Versions of subscribe()
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Subscribe to a topic, version for class member function with bare pointer
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe is a convenience function for using member functions, and can be used like so:
\verbatim
void Foo::callback(const std_msgs::Empty::ConstPtr& message)
{
}

Foo foo_object;
ros::Subscriber sub = handle.subscribe("my_topic", 1, &Foo::callback, &foo_object);
\endverbatim
   *
   * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param fp Member function pointer to call when a message has arrived
   * \param obj Object to call fp on
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj, 
                       const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /// and the const version
  template<class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M) const, T* obj, 
                       const TransportHints& transport_hints = TransportHints())
  {
    SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    return subscribe(ops);
  }

  /**
   * \brief Subscribe to a topic, version with full range of SubscribeOptions
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared pointer
   * to the message received.  This message should \b not be changed in place, as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe allows the full range of options, exposed through the SubscribeOptions class
   *
   * \param ops Subscribe options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  Subscriber subscribe(SubscribeOptions& ops);

};

}

#endif // ROSCPP_NODE_HANDLE_H

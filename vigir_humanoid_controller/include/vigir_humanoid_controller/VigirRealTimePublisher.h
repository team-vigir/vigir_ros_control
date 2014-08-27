/*=================================================================================================
// Copyright (c) 2013-2014, David Conner, TORC Robotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Team ViGIR or TORC Robotics nor the names of
//       its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This code is based on the paradigm established in ROS realtime_tools::RealtimeBuffer
//=================================================================================================
*/
#ifndef __VIGIR_REALTIME_PUBLISHER_H__
#define __VIGIR_REALTIME_PUBLISHER_H__

#include <time.h>
#include <atomic>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include <vigir_humanoid_controller/VigirRealTimeBuffer.h>

namespace vigir_control
{


// Base class for publishing real-time data in non-realtime thread
// Derive implmentation for a specific topic type, and extract message data from given DataType
// Use the VigirRealTimePublisher to publish all topics extracted from a given data type.
template<class DataType>
class VigirRealTimeTopicBase
{
public:

    VigirRealTimeTopicBase(const std::string& topic, const ros::Rate& rate, const bool& on_change)
        : topic_name_(topic), publish_rate_(rate), on_change_(on_change),next_publish_time_(0)
    {

        ROS_INFO("  parameters constructor for TopicBase");

    }

    virtual ~VigirRealTimeTopicBase(){}

    // Templated topic publisher that extracts topic specific data from common structure passed between real time threads
    virtual void publish(const DataType& data, const ros::Time& current_time) = 0;


protected:

    std::string     topic_name_;
    ros::Rate       publish_rate_;
    bool            on_change_;
    ros::Publisher  publisher_;
    ros::Time       next_publish_time_;


};


// Templated publisher that extracts messages from a given data type
template<typename DataType>
class VigirRealTimePublisher
{
 public:

    // Default constructor - (cannot use default publishLoop)
    VigirRealTimePublisher(const std::string& name = "VigirRealTimePublisher")
        :  keep_running_(true), publisher_name_(name)
    {
        sleep_ts_.tv_sec  = 0;
        sleep_ts_.tv_nsec = 50000; // 50 micro seconds
        ROS_INFO("Construct %s publisher ...",publisher_name_.c_str());
    }

    // Constructor that provides access to real time buffer for publishLoop
    VigirRealTimePublisher(const std::string& name, boost::shared_ptr<VigirRealTimeBuffer<DataType> >& rt_buffer)
        :  keep_running_(true), publisher_name_(name),rt_buffer_(rt_buffer)
    {
        sleep_ts_.tv_sec  = 0;
        sleep_ts_.tv_nsec = 50000; // 50 micro seconds
        ROS_INFO("Construct %s publisher with direct real time buffer access ...",publisher_name_.c_str());
    }

    ~VigirRealTimePublisher()
    {
        ROS_INFO("Destroy %s publisher ... ",publisher_name_.c_str());

    }

    const std::string& name(){return publisher_name_;}

    // Exit publishLoop
    void shutdown(){ keep_running_ = false;}


    // Add topic to be published
    void addTopic(typename boost::shared_ptr<VigirRealTimeTopicBase< DataType>  >& topic)
    {
        topics_.push_back(topic);
    }


    // Iterates through list of topics extracted from given data type
    inline void publish(const DataType& data, const ros::Time& current_time)
    {
        for (typename std::vector<boost::shared_ptr<VigirRealTimeTopicBase<DataType> > >::const_iterator it=topics_.begin(); it != topics_.end(); ++it)
        {
            (*it)->publish(data, current_time);
        }
    }

    void publishLoop()
    {

        if (!rt_buffer_)
        {
            ROS_ERROR("Cannot use publishLoop without setting a valid Real-time buffer object for data");
            throw;
        }


        ROS_INFO("Start publisher loop for %s",publisher_name_.c_str());
        DataType data;
        uint32_t last_data_count = std::numeric_limits<uint32_t>::max();
        timespec remaining_ts_;

        while (keep_running_ && ros::ok())
        {
            if (rt_buffer_->dataCount() != last_data_count)
            {
                last_data_count = rt_buffer_->readBuffer(data);
                publish(data,ros::Time::now());
            }
            else
            { // No new data since last loop
                nanosleep(&sleep_ts_, &remaining_ts_);

            }
        }
        ROS_INFO("Exit the publisher loop for %s",publisher_name_.c_str());
    }

 private:

    std::string                                                          publisher_name_;
    bool                                                                 keep_running_;
    std::vector< boost::shared_ptr<VigirRealTimeTopicBase< DataType> > > topics_;    // List of topics sharing a base data type
    boost::shared_ptr<VigirRealTimeBuffer<DataType> >                    rt_buffer_; // shared pointer to the real time buffer with data
    timespec                                                             sleep_ts_;

}; // class

}// namespace

#endif

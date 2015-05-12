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
#include <flor_utilities/timing.h>

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

    }

    virtual ~VigirRealTimeTopicBase()
    {
        //std::cout << "  Destroyed VigirRealTimeTopicBase for " << topic_name_ << "!" << std::endl;
    }

    // Templated topic publisher that extracts topic specific data from common structure passed between real time threads
    virtual bool publish(const DataType& data, const ros::Time& current_time) = 0;


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
        : keep_running_(true), publisher_name_(name),last_data_count_(std::numeric_limits<uint32_t>::max()),
          last_publish_time_(0),
          publisher_timing_(name,true,false)
    {
        //sleep_ts_.tv_sec  = 0;
        //sleep_ts_.tv_nsec = 50000; // 50 micro seconds
        ROS_INFO("Construct %s publisher ...",publisher_name_.c_str());
    }

    // Constructor that provides access to real time buffer for publishLoop
    VigirRealTimePublisher(const std::string& name, boost::shared_ptr<VigirRealTimeBuffer<DataType> >& rt_buffer)
        :  keep_running_(true), publisher_name_(name),rt_buffer_(rt_buffer), last_data_count_(std::numeric_limits<uint32_t>::max()),
           last_publish_time_(0),
           publisher_timing_(name,true,false)
    {
        //sleep_ts_.tv_sec  = 0;
        //sleep_ts_.tv_nsec = 50000; // 50 micro seconds
        ROS_INFO("Construct %s publisher with direct real time buffer access ...",publisher_name_.c_str());
    }

    ~VigirRealTimePublisher()
    {
        //std::cout << "      Destroyed " << publisher_name_ << " publisher!" << std::endl;
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
            if ( (*it)->publish(data, current_time) )
            { // if any published, then update time
                last_publish_time_ = current_time;
            }
        }
    }

    inline bool publishAnyNewData(const ros::Time& current_time = ros::Time::now())
    {
        if (rt_buffer_->dataCount() != last_data_count_)
        {
            DO_TIMING(publisher_timing_);
            last_data_count_ = rt_buffer_->readBuffer(last_data_);
            publish(last_data_,current_time);
            return true;
        }
        return false;
    }

    inline bool publishAnyNewDataNonBlocking(const ros::Time& current_time = ros::Time::now())
    {
        if (rt_buffer_->dataCount() != last_data_count_)
        {
            DO_TIMING(publisher_timing_);
            uint32_t data_cnt = rt_buffer_->readBufferNonBlocking(last_data_);
            if (data_cnt)
            {
                last_data_count_ = data_cnt;
                publish(last_data_,current_time);
            }
            return true;
        }
        return false;
    }

    inline bool publishLatestData(const ros::Duration& interval, const ros::Time& current_time = ros::Time::now())
    {
        if (rt_buffer_->dataCount() != last_data_count_)
        {
            DO_TIMING(publisher_timing_);
            last_data_count_ = rt_buffer_->readBuffer(last_data_);
            publish(last_data_,current_time);
            return true;
        }
        else if ( (current_time - last_publish_time_) > interval)
        {
            publish(last_data_, current_time);
            return true;
        }
        return false;
    }


    void publishLoop()
    {

        if (!rt_buffer_)
        {
            ROS_ERROR("Cannot use publishLoop without setting a valid Real-time buffer object for data");
            throw;
        }


        ROS_INFO("Start publisher loop for %s",publisher_name_.c_str());
        //timespec remaining_ts_;

        Timing loop_timing(publisher_name_+"Loop",true,false);

        while (keep_running_ && ros::ok())
        {
            DO_TIMING(loop_timing);
            publishAnyNewData();
            usleep(100); // release this thread as publishing ROS topics is not considered real time required
        }
        std::cout << "Exit the publisher loop for " << publisher_name_ << " !" << std::endl;
    }

 private:

    std::string                                                          publisher_name_;
    bool                                                                 keep_running_;
    std::vector< boost::shared_ptr<VigirRealTimeTopicBase< DataType> > > topics_;    // List of topics sharing a base data type
    boost::shared_ptr<VigirRealTimeBuffer<DataType> >                    rt_buffer_; // shared pointer to the real time buffer with data

    uint32_t                                                             last_data_count_;
    DataType                                                             last_data_;

    Timing                                                               publisher_timing_;
    ros::Time                                                            last_publish_time_;

    //timespec                                                             sleep_ts_;

}; // class

}// namespace

#endif


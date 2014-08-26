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
#ifndef __VIGIR_REALTIME_BUFFER_H__
#define __VIGIR_REALTIME_BUFFER_H__

#include <time.h>
#include <atomic>

namespace vigir_control
{

template <class T>
class VigirRealTimeBuffer
{
 public:

    // Default constructor
    VigirRealTimeBuffer()
      : lock_counter_(0), data_count_(std::numeric_limits<uint32_t>::max() )
    {
        // Define sleep for polling read lock
        sleep_ts_.tv_sec  = 0;
        sleep_ts_.tv_nsec = 20000; // 20 micro seconds

        // allocate memory
        read_data_buffer_  = new T();
        write_data_buffer_ = new T();
        swap_              = NULL;
    }

    ~VigirRealTimeBuffer()
    { // Free up the memory
          delete read_data_buffer_;
          delete write_data_buffer_;
    }

    // Copy constructor
    VigirRealTimeBuffer(VigirRealTimeBuffer &source)
        : lock_counter_(0), data_count_(std::numeric_limits<uint32_t>::max() )
    {
        // Define sleep for polling read lock
        sleep_ts_.tv_sec  = 0;
        sleep_ts_.tv_nsec = 20000; // 20 micro seconds

        // allocate memory
        read_data_buffer_  = new T();
        write_data_buffer_ = new T();
        swap_              = NULL;

        // Copy the data from old RTB to new RTB
        writeBuffer(source.getConstReference());
        writeBuffer(source.getConstReference()); // write twice to initialize both buffers
    }

    // Data constructor
    VigirRealTimeBuffer(const T &source)
        : lock_counter_(0), data_count_(std::numeric_limits<uint32_t>::max() )
    {
        // Define sleep for polling read lock
        sleep_ts_.tv_sec  = 0;
        sleep_ts_.tv_nsec = 20000; // 20 micro seconds

        // allocate memory
        read_data_buffer_  = new T();
        write_data_buffer_ = new T();
        swap_              = NULL;

        // Initialize buffer from the data source
        writeBuffer(source);
        writeBuffer(source); // write twice to initialize both buffers
    }

    /*!
    * @brief Custom assignment operator
    */
    VigirRealTimeBuffer &operator =(const VigirRealTimeBuffer& source)
    {
        if (this == &source)
          return *this;
        std::cout << "copy  RTB constructor" << std::endl;

        // Copy the data from source buffer to new buffer
        writeBuffer(*source.readBuffer());

        return *this;
    }

    VigirRealTimeBuffer &operator =(const T& source)
    {

        // Copy the data from source buffer to new buffer
        writeBuffer(source);

        return *this;
    }

    uint32_t readBuffer(T& data)
    {

        // Check if the data is currently being written to (is locked)
        while(!data_mutex_.try_lock_shared())
        {
            ++lock_counter_; // debug monitoring
            nanosleep(&sleep_ts_, &remaining_ts_);
        }

        // --------- debug ----------
        if (lock_counter_ > 1)
        {   // Debug status to we ever get blocked for more than 1 cycle
            std::cout << "RTB lock counter = " << lock_counter_ << std::endl;
        }
        // -------------------------------------------


        // Copy the data from the read buffer
        data = *read_data_buffer_;
        uint32_t   count = data_count_;
        lock_counter_ = 0;           // debug lock counter that will print warning if we are blocked for more than 20 us
        data_mutex_.unlock_shared(); // free the lock now that we've copied data

        return count; // return data count associated with this data

    }


    // return the count of the data transfer to help readers track when new data is available
    uint32_t dataCount() { return data_count_.load();}

    // Provide accessor functions to allow interacting with write buffer
    const T& getConstReference(){return *write_data_buffer_;}
    T& getWritableReference() { return *write_data_buffer_;}
    T* getWriteablePtr() { return  write_data_buffer_;}

    // Set read buffer after updating the write buffer
    inline void setReadBuffer()
    {
      // get upgradable access
      boost::upgrade_lock<boost::shared_mutex> lock(data_mutex_);

      // get exclusive access to block the shared readers
      boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
      data_count_++; // increment counter to show that data is changed

      // Swap the read and write buffer pointers while uniquely locked
      swap_               = read_data_buffer_;
      read_data_buffer_   = write_data_buffer_;
      write_data_buffer_  = swap_;

    }

  void writeBuffer(const T& data)
  {

      // Copy the data to writeable buffer
      *write_data_buffer_ = data;
      setReadBuffer();
  }


  volatile std::atomic_uint data_count_;

 private:

  T* read_data_buffer_;
  T* write_data_buffer_;
  T* swap_;

  timespec sleep_ts_;
  timespec remaining_ts_;
  // Set as mutable for read buffer
  boost::shared_mutex data_mutex_;

  // Debug monitoring to see if read buffer is every blocked more than once
  uint lock_counter_;

}; // class

}// namespace

#endif


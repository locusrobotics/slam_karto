/***************************************************************************
 * Copyright (C) 2017 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#include <slam_karto/loop_closure_callback.h>

#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include <string>


namespace slam_karto
{

LoopClosureCallback::LoopClosureCallback(double min_loop_closure_time) :
    karto::MapperLoopClosureListener(),
    min_loop_closure_duration_(min_loop_closure_time),
    loop_closure_start_time_(0, 0),
    loop_closure_duration_(0, 0)
{
}

LoopClosureCallback::~LoopClosureCallback()
{
}

void LoopClosureCallback::RegisterLoopClosureCheckCallback(Callback callback)
{
  check_callbacks_.push_back(callback);
}

void LoopClosureCallback::RegisterBeginLoopClosureCallback(Callback callback)
{
  begin_callbacks_.push_back(callback);
}

void LoopClosureCallback::RegisterEndLoopClosureCallback(Callback callback)
{
  end_callbacks_.push_back(callback);
}

void LoopClosureCallback::LoopClosureCheck(const std::string& rInfo)
{
  // Call all registered callbacks
  for (size_t i = 0; i < check_callbacks_.size(); ++i)
  {
    check_callbacks_.at(i)(rInfo);
  }
}

void LoopClosureCallback::BeginLoopClosure(const std::string& rInfo)
{
  // Store the timestamp of the loop closure start. Using WallTime to handle bagfile playback cases.
  loop_closure_start_time_ = ros::WallTime::now();

  // Check if the last loop closure took long enough to trigger the callbacks
  if (loop_closure_duration_ >= min_loop_closure_duration_)
  {
    // Call all registered callbacks
    for (size_t i = 0; i < begin_callbacks_.size(); ++i)
    {
      begin_callbacks_.at(i)(rInfo);
    }
  }
}

void LoopClosureCallback::EndLoopClosure(const std::string& rInfo)
{
  // Check if the last loop closure took long enough to trigger the callbacks
  if (loop_closure_duration_ >= min_loop_closure_duration_)
  {
    // Call all registered callbacks
    for (size_t i = 0; i < end_callbacks_.size(); ++i)
    {
      end_callbacks_.at(i)(rInfo);
    }
  }

  // Compute the loop closure duration. Using WallTime to handle bagfile playback cases.
  ros::WallTime loop_closure_end_time = ros::WallTime::now();
  if (loop_closure_start_time_ != ros::WallTime(0, 0))
  {
    loop_closure_duration_ = loop_closure_end_time - loop_closure_start_time_;
  }

  ROS_INFO_STREAM("Loop closure processing time: " << loop_closure_duration_);
}

}  // namespace slam_karto

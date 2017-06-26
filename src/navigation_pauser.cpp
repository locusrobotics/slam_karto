/***************************************************************************
 * Copyright (C) 2017 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#include "navigation_pauser.h"

#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include <string>


NavigationPauser::NavigationPauser(
  const ros::NodeHandle& node_handle,
  const ros::NodeHandle& private_node_handle) :
    karto::MapperLoopClosureListener(),
    node_handle_(node_handle),
    min_loop_closure_duration_(0.10),
    loop_closure_start_time_(0, 0),
    loop_closure_duration_(0, 0)
{
  // Configure the Navigation Pauser from the parameter server
  double min_loop_closure_duration;
  if (private_node_handle.getParam("min_loop_closure_duration",  min_loop_closure_duration))
  {
    min_loop_closure_duration_.fromSec(min_loop_closure_duration);
  }

  // Advertise the pause topic
  pause_publisher_ = node_handle_.advertise<std_msgs::Bool>("pause_topic", 1, true);

  // Connect to a pause service server
  pause_service_client_ = node_handle_.serviceClient<std_srvs::SetBool>("pause_service", false);
}

NavigationPauser::~NavigationPauser()
{
}

void NavigationPauser::BeginLoopClosure(const std::string& rInfo)
{
  // Store the timestamp of the loop closure start. Using WallTime to handle bagfile playback cases.
  loop_closure_start_time_ = ros::WallTime::now();

  // Check if the last loop closure took long enough to trigger a navigation pause
  if (loop_closure_duration_ < min_loop_closure_duration_)
  {
    return;
  }

  // Publish a pause navigation message
  if (pause_publisher_.getNumSubscribers() > 0)
  {
    ROS_INFO_STREAM("Publishing pause navigation message...");
    std_msgs::Bool msg;
    msg.data = true;
    pause_publisher_.publish(msg);
  }

  // Call the pause navigation service
  if (pause_service_client_.exists())
  {
    ROS_INFO_STREAM("Calling pause navigation service...");
    std_srvs::SetBool srv;
    srv.request.data = true;
    pause_service_client_.call(srv);
  }
}

void NavigationPauser::EndLoopClosure(const std::string& rInfo)
{
  // Compute the loop closure duration. Using WallTime to handle bagfile playback cases.
  ros::WallTime loop_closure_end_time = ros::WallTime::now();
  if (loop_closure_start_time_ != ros::WallTime(0, 0))
  {
    loop_closure_duration_ = loop_closure_end_time - loop_closure_start_time_;
    ROS_INFO_STREAM("Loop closure processing time: " << loop_closure_duration_);
  }

  // Publish a resume navigation message
  if (pause_publisher_.getNumSubscribers() > 0)
  {
    ROS_INFO_STREAM("Publishing resume navigation message...");
    std_msgs::Bool msg;
    msg.data = false;
    pause_publisher_.publish(msg);
  }

  // Call the pause navigation service
  if (pause_service_client_.exists())
  {
    ROS_INFO_STREAM("Calling resume navigation service...");
    std_srvs::SetBool srv;
    srv.request.data = false;
    pause_service_client_.call(srv);
  }
}

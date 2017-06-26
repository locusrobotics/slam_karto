/***************************************************************************
 * Copyright (C) 2017 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#ifndef KARTO_NAVIGATION_PAUSER_H
#define KARTO_NAVIGATION_PAUSER_H

#include <open_karto/Mapper.h>
#include <ros/ros.h>
#include <string>

/**
 * Class that listens for the start of a loop closure and pauses navigation. It resumes
 * navigation once the loop closure is complete.
 */
class NavigationPauser : public karto::MapperLoopClosureListener
{
public:
  /**
   * Constructor
   */
  explicit NavigationPauser(
    const ros::NodeHandle& node_handle = ros::NodeHandle(),
    const ros::NodeHandle& private_node_handle = ros::NodeHandle("~"));

  /**
   * Destructor
   */
  virtual ~NavigationPauser();

  /**
   * Called when loop closure is starting. If the previous loop closure processing time exceeded the configured
   * threshold, then a pause navigation message is published if anyone is listeneing, and the pause navigation service
   * is called if it exists.
   */
  virtual void BeginLoopClosure(const std::string& rInfo);

  /**
   * Called when loop closure is over. This publishes a resume navigation message if anyone is listening and calls
   * the resume navigation service if it exists.
   */
  virtual void EndLoopClosure(const std::string& rInfo);

protected:
  ros::NodeHandle node_handle_;  //!< A node handle in the root namespace
  ros::Publisher pause_publisher_;  //!< Topic that publishes requests to pause/unpause navigation
  ros::ServiceClient pause_service_client_;  //!< Service client that requests to pause/unpause navigation
  ros::WallDuration min_loop_closure_duration_;  //!< Only pause navigation is the last loop closure took this long
  ros::WallTime loop_closure_start_time_;  //!< The timestamp of the last BeginLoopClosure signal
  ros::WallDuration loop_closure_duration_;  //!< The time required for the last loop closure to complete
  // Note: Use WallTime to handle the bagfile playback case. If the bagfiles are paused, then time does not advance and
  // the loop closure duration will always be nearly zero.
};  // NavigationPauseListener

#endif  // KARTO_NAVIGATION_PAUSER_H

/***************************************************************************
 * Copyright (C) 2017 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#ifndef SLAM_KARTO_LOOP_CLOSURE_CALLBACK_H
#define SLAM_KARTO_LOOP_CLOSURE_CALLBACK_H

#include <open_karto/Mapper.h>
#include <ros/ros.h>
#include <string>
#include <vector>


namespace slam_karto
{

/**
 * Class that listens for the start and end of loop closures and calls the provided functions.
 */
class LoopClosureCallback : public karto::MapperLoopClosureListener
{
public:
  typedef boost::function<void(const std::string&)> Callback;

  /**
   * Constructor
   *
   * @param min_loop_closure_time The minimum cpu time of a loop closure to trigger the BeginLoopClosure callbacks.
   *                              This allows the callbacks to be triggered only for "long" loop closures. The cpu
   *                              time is computed on the previous loop closure, so this is a lagging property.
   */
  explicit LoopClosureCallback(double min_loop_closure_time = 0.0);

  /**
   * Destructor
   */
  virtual ~LoopClosureCallback();

  /**
   * @brief Add a callback to be triggered whenever a loop closure starts
   */
  void RegisterLoopClosureCheckCallback(Callback callback);

  /**
   * @brief Add a callback to be triggered whenever a loop closure starts
   */
  void RegisterBeginLoopClosureCallback(Callback callback);

  /**
   * @brief Add a callback to be triggered whenever a loop closure starts
   */
  void RegisterEndLoopClosureCallback(Callback callback);

  /**
   * Triggers all registered callbacks whenever the system starts checking for a loop closure
   */
  virtual void LoopClosureCheck(const std::string& rInfo);

  /**
   * Triggers all registered callbacks when a "long" loop closure is starting.
   */
  virtual void BeginLoopClosure(const std::string& rInfo);

  /**
   * Triggers all registered callbacks when a "long" loop closure is complete.
   */
  virtual void EndLoopClosure(const std::string& rInfo);

protected:
  std::vector<Callback> check_callbacks_;  //!< Callbacks to trigger whenever the system checks for loop closures
  std::vector<Callback> begin_callbacks_;  //!< Callbacks to trigger whenever the system starts a loop closures
  std::vector<Callback> end_callbacks_;  //!< Callbacks to trigger whenever the system completes a loop closures
  ros::WallDuration min_loop_closure_duration_;  //!< Only pause navigation is the last loop closure took this long
  ros::WallTime loop_closure_start_time_;  //!< The timestamp of the last BeginLoopClosure signal
  ros::WallDuration loop_closure_duration_;  //!< The time required for the last loop closure to complete
  // Note: Use WallTime to handle the bagfile playback case. If the bagfiles are paused, then time does not advance and
  // the loop closure duration will always be nearly zero.
};  // LoopClosurePauser

}  // namespace slam_karto

#endif  // SLAM_KARTO_LOOP_CLOSURE_CALLBACK_H

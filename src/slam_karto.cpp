/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

/**

@mainpage karto_gmapping

@htmlinclude manifest.html

*/

#include "ros/ros.h"
#include "ros/console.h"
#include "message_filters/subscriber.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "visualization_msgs/MarkerArray.h"

#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include "open_karto/Mapper.h"

#include <slam_karto/loop_closure_callback.h>
#include "spa_solver.h"

#include <boost/algorithm/clamp.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

class SlamKarto
{
  public:
    SlamKarto();
    ~SlamKarto();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);

    /**
    * @brief Indicates if the system is (supposed to be) paused
    */
    bool isPaused() const { return is_paused_; }

    /**
    * @brief Send all of the pause navigation signals
    */
    void pauseNavigation();

    /**
    * @brief Send all of the resume navigation signals
    */
    void resumeNavigation();

    /**
     * @brief Returns the percentage filled of the scan queue in the range [0.0, 1.0]
     */
    double queueFillPercentage() const { return static_cast<double>(scan_queue_.size()) / scan_queue_.capacity(); }

  private:
    bool getOdomPose(karto::Pose2& karto_pose, const ros::Time& t);
    karto::LaserRangeFinder* getLaser(const sensor_msgs::LaserScan::ConstPtr& scan);

    /**
     * @brief Convert a ROS sensor_msgs::Laserscan into a karto::LocalizedRangeScan
     */
    karto::LocalizedRangeScan* convertScan(karto::LaserRangeFinder* laser,
      const sensor_msgs::LaserScan::ConstPtr& scan);

    /**
     * @brief Updates the map->odom frame transform using the optimized pose of the provided range scan
     */
    void updateMapToOdomTransform(karto::LocalizedRangeScan* range_scan);

    /**
     * @brief Checks if the pose of the provided scan is sufficiently different from the previous
     * scan to be added to the pose graph.
     */
    bool hasMovedEnough(const karto::LocalizedRangeScan* scan);

    /**
     * @brief Publish the most recent map->odom transform with the current time.
     */
    void publishTransform();

    /**
     * @brief Thread function for publishing the map->odom transform at a regular interval
     *
     * Publishing in a separate thread ensures the transform timestamp is always recent, allowing tf to work
     * correctly.
     */
    void publishLoop(double transform_publish_period);

    /**
     * @brief Copy the scans from the mapper and build a new map.
     *
     * Note that this function blocks, waiting for access to the mapper's set of laserscans.
     */
    bool updateMap();

    /**
     * @brief Thread function for building and publishing the map
     *
     * Running the map generation code in a separate thread allows the mapping process to (a) wait for the
     * optimization thread to complete without affecting the servicing of ROS callbacks, and (b) allow the
     * map building process as much time as needed to complete without interfering with the optimization or
     * ROS callbacks. The map building process can take a long time (>1s), even for moderately sized spaces.
     */
    void mapLoop(double map_update_interval);

    /**
     * @brief Publish an updated visualization of the pose graph
     *
     * Note that this function blocks, waiting for access to the mapper's graph.
     */
    void publishGraphVisualization();

    /**
     * @brief Thread function for updating the pose graph
     *
     * The received laserscans are queued for the processing in the main ROS callback thread. In this thread, the
     * queued laserscans are processed by the Karto mapper one at a time. This prevents the system from dropping
     * scans during long runing Karto updates.
     */
    void optimizationLoop();

    /**
     * @brief Display the queue fill precentage above the robot
     */
    void publishQueueVisualization();

    // ROS handles
    ros::NodeHandle node_;
    tf::TransformListener tf_;
    tf::TransformBroadcaster* tfB_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    ros::Publisher sst_;
    ros::Publisher marker_publisher_;
    ros::Publisher sstm_;
    ros::ServiceServer ss_;
    ros::Publisher pause_publisher_;  //!< Topic that publishes requests to pause/unpause navigation
    ros::ServiceClient pause_service_client_;  //!< Service client that requests to pause/unpause navigation

    // The map that will be published / send to service callers
    nav_msgs::GetMap::Response map_;

    // Storage for ROS parameters
    std::string aligned_frame_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
    int throttle_scans_;
    double resolution_;
    bool pause_on_loop_closure_;  //!< Issue pause/resume commands in response to loop closure events
    bool pause_on_full_queue_;  //!< Issue pause/resume navigation commands in response to the queue size
    double pause_navigation_percentage_;  //!< Only pause navigation when the queue is more than this full (0.0, 1.0)
    double resume_navigation_percentage_;  //!< Only resume navigation when the queue is less than this full (0.0, 1.0)
    boost::mutex map_mutex_;
    boost::mutex map_to_odom_mutex_;
    boost::mutex mapper_mutex_;

    // Karto bookkeeping
    karto::Mapper* mapper_;
    karto::Dataset* dataset_;
    SpaSolver* solver_;
    slam_karto::LoopClosureCallback* loop_closure_pauser_;  //!< Listen to loop closure events from karto and pause nav
    std::map<std::string, karto::LaserRangeFinder*> lasers_;
    std::map<std::string, bool> lasers_inverted_;

    // Internal state
    bool got_map_;
    int laser_count_;
    boost::thread* transform_thread_;  //!< Separate thread for publishing the map->odom transformation.
    boost::thread* map_thread_;  //!< Separate thread for building the map image/occupancy grid.
    boost::thread* optimization_thread_;  //!< Separate thread running the Karto mapper optimizations.
    tf::Transform map_to_odom_;
    unsigned marker_count_;
    bool inverted_laser_;
    bool is_paused_;  //!< Flag indicating the system is (supposed to be) paused
    bool first_scan_received_;  //!< Flag to track if the first scan was received
    double last_scan_time_;  //!< The timestamp of the most recently queued range scan
    karto::Pose2 last_scan_pose_;  //!< The odom frame pose of the most recently queued range scan

    // Laserscan queue for to-be-processed scans
    boost::circular_buffer<karto::LocalizedRangeScan*> scan_queue_;  //!< Fixed-sized buffer for storing range scans
                                                                     //!< that have not been processed by the mapper yet
    boost::condition_variable scan_queue_data_available_;  //!< Variable used to synchronize the producer and consumer
                                                           //!< threads without a busy-wait queue size check
    boost::mutex scan_queue_mutex_;  //!< Mutex lock for queue operations (push, pop, front)
};

SlamKarto::SlamKarto() :
        pause_on_loop_closure_(false),
        loop_closure_pauser_(NULL),
        got_map_(false),
        laser_count_(0),
        transform_thread_(NULL),
        map_thread_(NULL),
        optimization_thread_(NULL),
        marker_count_(0),
        is_paused_(false),
        tf_(ros::Duration(60.0)),
        scan_queue_(1),
        first_scan_received_(false),
        last_scan_time_(0.0),
        last_scan_pose_(0, 0, 0)
{
  map_to_odom_.setIdentity();
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("aligned_frame", aligned_frame_))
    aligned_frame_ = "map_aligned";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  double map_update_interval;
  private_nh_.param("map_update_interval", map_update_interval, 5.0);
  if(!private_nh_.getParam("resolution", resolution_))
  {
    // Compatibility with slam_gmapping, which uses "delta" to mean
    // resolution
    if(!private_nh_.getParam("delta", resolution_))
      resolution_ = 0.05;
  }
  double transform_publish_period;
  private_nh_.param("transform_publish_period", transform_publish_period, 0.05);
  private_nh_.param("pause_on_loop_closure", pause_on_loop_closure_, false);
  private_nh_.param("pause_on_full_queue", pause_on_full_queue_, false);
  private_nh_.param("pause_navigation_percentage", pause_navigation_percentage_, 0.90);
  private_nh_.param("resume_navigation_percentage_", resume_navigation_percentage_, 0.10);
  pause_navigation_percentage_ = boost::algorithm::clamp(pause_navigation_percentage_, 0.0, 1.0);
  resume_navigation_percentage_ = boost::algorithm::clamp(resume_navigation_percentage_,
    0.0, pause_navigation_percentage_);
  int scan_queue_length;
  if (private_nh_.getParam("scan_queue_length", scan_queue_length))
  {
    if (scan_queue_length > 0)
    {
      scan_queue_.set_capacity(scan_queue_length);
    }
    else
    {
      ROS_WARN_STREAM("Parameter scan_queue_length must be greater than zero. Ignoring.");
    }
  }

  // Set up advertisements and subscriptions
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamKarto::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamKarto::laserCallback, this, _1));
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  pause_publisher_ = node_.advertise<std_msgs::Bool>("pause_topic", 1, true);
  pause_service_client_ = node_.serviceClient<std_srvs::SetBool>("pause_service", false);

  // Initialize Karto structures
  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();
  if (pause_on_loop_closure_)
  {
    double min_loop_closure_duration;
    private_nh_.param("min_loop_closure_duration", min_loop_closure_duration, 0.0);
    loop_closure_pauser_ = new slam_karto::LoopClosureCallback(min_loop_closure_duration);
    loop_closure_pauser_->RegisterBeginLoopClosureCallback(boost::bind(&SlamKarto::pauseNavigation, this));
    loop_closure_pauser_->RegisterEndLoopClosureCallback(boost::bind(&SlamKarto::resumeNavigation, this));
    mapper_->AddListener(loop_closure_pauser_);
  }

  // Setting General Parameters from the Parameter Server
  bool use_scan_matching;
  if(private_nh_.getParam("use_scan_matching", use_scan_matching))
    mapper_->setParamUseScanMatching(use_scan_matching);
  
  bool use_scan_barycenter;
  if(private_nh_.getParam("use_scan_barycenter", use_scan_barycenter))
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  double minimum_travel_distance;
  if(private_nh_.getParam("minimum_travel_distance", minimum_travel_distance))
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

  double minimum_travel_heading;
  if(private_nh_.getParam("minimum_travel_heading", minimum_travel_heading))
    mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  int scan_buffer_size;
  if(private_nh_.getParam("scan_buffer_size", scan_buffer_size))
    mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance;
  if(private_nh_.getParam("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
    mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine;
  if(private_nh_.getParam("link_match_minimum_response_fine", link_match_minimum_response_fine))
    mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  double link_scan_maximum_distance;
  if(private_nh_.getParam("link_scan_maximum_distance", link_scan_maximum_distance))
    mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance;
  if(private_nh_.getParam("loop_search_maximum_distance", loop_search_maximum_distance))
    mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing;
  if(private_nh_.getParam("do_loop_closing", do_loop_closing))
    mapper_->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size;
  if(private_nh_.getParam("loop_match_minimum_chain_size", loop_match_minimum_chain_size))
    mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse;
  if(private_nh_.getParam("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse))
    mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse;
  if(private_nh_.getParam("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse))
    mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine;
  if(private_nh_.getParam("loop_match_minimum_response_fine", loop_match_minimum_response_fine))
    mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters from the Parameter Server

  double correlation_search_space_dimension;
  if(private_nh_.getParam("correlation_search_space_dimension", correlation_search_space_dimension))
    mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution;
  if(private_nh_.getParam("correlation_search_space_resolution", correlation_search_space_resolution))
    mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation;
  if(private_nh_.getParam("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter Server

  double loop_search_space_dimension;
  if(private_nh_.getParam("loop_search_space_dimension", loop_search_space_dimension))
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution;
  if(private_nh_.getParam("loop_search_space_resolution", loop_search_space_resolution))
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation;
  if(private_nh_.getParam("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
    mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters from the Parameter Server

  double distance_variance_penalty;
  if(private_nh_.getParam("distance_variance_penalty", distance_variance_penalty))
    mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty;
  if(private_nh_.getParam("angle_variance_penalty", angle_variance_penalty))
    mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset;
  if(private_nh_.getParam("fine_search_angle_offset", fine_search_angle_offset))
    mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset;
  if(private_nh_.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
    mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution;
  if(private_nh_.getParam("coarse_angle_resolution", coarse_angle_resolution))
    mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty;
  if(private_nh_.getParam("minimum_angle_penalty", minimum_angle_penalty))
    mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty;
  if(private_nh_.getParam("minimum_distance_penalty", minimum_distance_penalty))
    mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion;
  if(private_nh_.getParam("use_response_expansion", use_response_expansion))
    mapper_->setParamUseResponseExpansion(use_response_expansion);

  // Set solver to be used in loop closure
  solver_ = new SpaSolver();

  std::string spa_method_string;
  int spa_method = SBA_SPARSE_CHOLESKY;

  if(private_nh_.getParam("spa_method", spa_method_string))
  {
    if(spa_method_string == "dense_cholesky")
      spa_method = SBA_DENSE_CHOLESKY;
    else if(spa_method_string == "gradient")
      spa_method = SBA_GRADIENT;
    else if(spa_method_string == "block_jacobian_pcg")
      spa_method = SBA_BLOCK_JACOBIAN_PCG;
    else if(spa_method_string != "sparse_cholesky")
      ROS_WARN_STREAM("\"" << spa_method_string << "\" is an invalid spa_parameter value. Valid values are "
          "\"sparse_cholesky,\" \"dense_cholesky,\" \"gradient,\" and \"block_jacobian_pcg.\" "
          "Assuming sparse_cholesky.");
  }

  solver_->SetSpaMethod(spa_method);
  mapper_->SetScanSolver(solver_);

  // Create a thread to periodically publish the latest map->odom
  // transform; it needs to go out regularly, uninterrupted by potentially
  // long periods of computation in our main loop.
  transform_thread_ = new boost::thread(boost::bind(&SlamKarto::publishLoop, this, transform_publish_period));

  // Create a thread to periodically rebuild the map from the laserscans.
  // The map is not used by karto itself, so it may be built in parallel
  // without affecting the actual algorithm.
  map_thread_ = new boost::thread(boost::bind(&SlamKarto::mapLoop, this, map_update_interval));

  // Create a thread for running the Karto scan processor. This allows the ROS callback
  // system to continue to run, adding new scans to the scan queue for Karto to process
  // when it has time. This prevents Karto from skipping scans when it gets busy.
  optimization_thread_ = new boost::thread(
    boost::bind(&SlamKarto::optimizationLoop, this));
}

SlamKarto::~SlamKarto()
{
  // Notify the queue condition variable so it will wake up from its sleep
  scan_queue_data_available_.notify_all();
  // Shutdown all of the threads
  if(transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  if (map_thread_)
  {
    map_thread_->join();
    delete map_thread_;
  }
  if (optimization_thread_)
  {
    optimization_thread_->join();
    delete optimization_thread_;
  }
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
  if (solver_)
    delete solver_;
  if (mapper_)
    delete mapper_;
  if (dataset_)
    delete dataset_;
  for (std::map<std::string, karto::LaserRangeFinder*>::iterator iter = lasers_.begin();
       iter != lasers_.end();
       ++iter)
  {
    delete iter->second;
  }
  lasers_.clear();
  if (loop_closure_pauser_)
    delete loop_closure_pauser_;
  // Delete any pending laserscans
  while (!scan_queue_.empty())
  {
    karto::LocalizedRangeScan* range_scan = scan_queue_.front();
    scan_queue_.pop_front();
    delete range_scan;
  }
}

void
SlamKarto::optimizationLoop()
{
  // Continue processing as long as ROS is running
  while (ros::ok())
  {
    // Wait for data to arrive. The while-loop guards against spurious wakeups.
    boost::mutex::scoped_lock scan_queue_lock(scan_queue_mutex_);
    while (ros::ok() && scan_queue_.empty())
    {
      scan_queue_data_available_.wait(scan_queue_lock);
    }
    // Check if we are shutting down
    if (!ros::ok())
    {
      return;
    }
    // The scan_queue_mutex is locked at this point.
    // Resume navigation when the queue drops below the fill threshold
    if (pause_on_full_queue_ && isPaused() && queueFillPercentage() < resume_navigation_percentage_)
    {
      resumeNavigation();
    }
    // Get the next laser scan off the queue and unlock so ROS can continue filling the buffer.
    karto::LocalizedRangeScan* range_scan = scan_queue_.front();
    scan_queue_.pop_front();
    scan_queue_lock.unlock();
    // But now we need to use the karto mapper. Acquire a lock for it before modifying the graph.
    bool processed = false;
    {
      boost::mutex::scoped_lock lock(mapper_mutex_);
      // Finally, process the scan with karto
      processed = mapper_->Process(range_scan);
    }
    // If this scan was successfully processed, then update the tf map->odom transform
    if (processed)
    {
      // Update the map->odom transform using this scan's optimized pose
      updateMapToOdomTransform(range_scan);
      // Add the localized range scan to the dataset (for memory management)
      dataset_->Add(range_scan);
    }
    else
    {
      // We are not using this scan. Delete it now.
      delete range_scan;
    }
  }
}

void
SlamKarto::updateMapToOdomTransform(karto::LocalizedRangeScan* range_scan)
{
  // Look up the odom->base transform
  karto::Pose2 odom_to_base_pose = range_scan->GetOdometricPose();
  tf::Transform odom_to_base_transform(
    tf::createQuaternionFromRPY(0, 0, odom_to_base_pose.GetHeading()),
    tf::Vector3(odom_to_base_pose.GetX(), odom_to_base_pose.GetY(), 0.0));
  // Look up the map->base transform
  karto::Pose2 map_to_base_pose = range_scan->GetCorrectedPose();
  tf::Transform map_to_base_transform(
    tf::createQuaternionFromRPY(0, 0, map_to_base_pose.GetHeading()),
    tf::Vector3(map_to_base_pose.GetX(), map_to_base_pose.GetY(), 0.0));
  // Compute the map->odom transform as map->base * base->odom
  {
    boost::mutex::scoped_lock lock(map_to_odom_mutex_);
    map_to_odom_ = map_to_base_transform * odom_to_base_transform.inverse();
  }
}

void
SlamKarto::publishLoop(double transform_publish_period)
{
  if(transform_publish_period == 0)
    return;

  ros::WallRate r(1.0 / transform_publish_period);
  while(ros::ok())
  {
    publishTransform();
    r.sleep();
  }
}

void
SlamKarto::publishTransform()
{
  boost::mutex::scoped_lock lock(map_to_odom_mutex_);
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
  tfB_->sendTransform(tf::StampedTransform (map_to_odom_, ros::Time::now(), map_frame_, odom_frame_));
}

karto::LaserRangeFinder*
SlamKarto::getLaser(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Check whether we know about this laser yet
  if(lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try
    {
      tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch(tf::TransformException e)
    {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
	       e.what());
      return NULL;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
	     scan->header.frame_id.c_str(),
	     laser_pose.getOrigin().x(),
	     laser_pose.getOrigin().y(),
	     yaw);
    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser frame
    // if the point's z-value is <=0, it is upside-down

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);

    try
    {
      tf_.transformPoint(scan->header.frame_id, up, up);
      ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse)
      ROS_INFO("laser is mounted upside-down");


    // Create a laser range finder device and copy in data from the first
    // scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder* laser = 
      karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
				      laser_pose.getOrigin().y(),
				      yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    // TODO: expose this, and many other parameters
    //laser_->SetRangeThreshold(12.0);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

bool
SlamKarto::getOdomPose(karto::Pose2& karto_pose, const ros::Time& t)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                           tf::Vector3(0,0,0)), t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  karto_pose = 
          karto::Pose2(odom_pose.getOrigin().x(),
                       odom_pose.getOrigin().y(),
                       yaw);
  return true;
}

void
SlamKarto::publishGraphVisualization()
{
  // Only compute the visualization marker if someone is listening
  if (marker_publisher_.getNumSubscribers() > 0)
  {
    // Copy the graph from the solver, limiting the time the solver
    // must be locked
    std::vector<float> graph;
    {
      boost::mutex::scoped_lock lock(mapper_mutex_);
      solver_->getGraph(graph);  // The solver is used by the mapper, so we lock the mapper mutex to ensure access
    }

    visualization_msgs::Marker nodes;
    nodes.header.frame_id = map_frame_;
    nodes.header.stamp = ros::Time::now();
    nodes.ns = "karto";
    nodes.id = 0;
    nodes.action = visualization_msgs::Marker::ADD;
    nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    nodes.pose.position.x = 0.0;
    nodes.pose.position.y = 0.0;
    nodes.pose.position.z = 0.0;
    nodes.scale.x = 0.1;
    nodes.scale.y = 0.1;
    nodes.scale.z = 0.1;
    nodes.color.r = 1.0;
    nodes.color.g = 0.0;
    nodes.color.b = 0.0;
    nodes.color.a = 1.0;
    nodes.lifetime = ros::Duration(0);

    visualization_msgs::Marker edges;
    edges.header.frame_id = map_frame_;
    edges.header.stamp = ros::Time::now();
    edges.ns = "karto";
    edges.id = 1;
    edges.action = visualization_msgs::Marker::ADD;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.scale.x = 0.035;
    edges.scale.y = 0.035;
    edges.scale.z = 0.035;
    edges.color.r = 0.0;
    edges.color.g = 0.0;
    edges.color.b = 1.0;
    edges.color.a = 1.0;
    edges.lifetime = ros::Duration(0);

    std::set< std::pair<double, double> > nodes_points;
    for (uint i = 0; i < graph.size() / 4; i++)
    {
      // Convert the graph into a set of linked poses
      geometry_msgs::Point point1;
      point1.x = graph[4 * i + 0];
      point1.y = graph[4 * i + 1];
      geometry_msgs::Point point2;
      point2.x = graph[4 * i + 2];
      point2.y = graph[4 * i + 3];

      // Store each pose in a sorted container to remove duplicates.
      // The ROS messages do not provide comparison operators to do
      // this with message types in a container.
      nodes_points.insert(std::pair<double, double>(point1.x, point1.y));
      nodes_points.insert(std::pair<double, double>(point2.x, point2.y));

      // Add each pair of poses as an edge in the line list
      edges.points.push_back(point1);
      edges.points.push_back(point2);
    }

    // Add each unique pose as a point in the sphere list
    for (std::set<std::pair<double, double> >::const_iterator it = nodes_points.begin(); it != nodes_points.end(); ++it)
    {
      geometry_msgs::Point point;
      point.x = it->first;
      point.y = it->second;
      nodes.points.push_back(point);
    }

    // Create a single marker array message from the nodes and edges markers
    visualization_msgs::MarkerArray marray;
    marray.markers.push_back(nodes);
    marray.markers.push_back(edges);

    marker_publisher_.publish(marray);
  }
}

void
SlamKarto::publishQueueVisualization()
{
  // Publish queue status
  if (scan_queue_.capacity() > 1 && marker_publisher_.getNumSubscribers() > 0)
  {
    visualization_msgs::Marker queue_size;
    queue_size.header.frame_id = base_frame_;
    queue_size.header.stamp = ros::Time::now();
    queue_size.ns = "karto_scan_queue";
    queue_size.id = 0;
    queue_size.action = visualization_msgs::Marker::ADD;
    queue_size.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    queue_size.pose.position.x = 0.0;
    queue_size.pose.position.y = 0.40;
    queue_size.pose.position.z = 0.0;
    queue_size.scale.x = 0.25;
    queue_size.scale.y = 0.25;
    queue_size.scale.z = 0.25;
    queue_size.color.r = 0.0;
    queue_size.color.g = 0.0;
    queue_size.color.b = 0.0;
    queue_size.color.a = 1.0;
    queue_size.text = "queue: " + boost::lexical_cast<std::string>(static_cast<int>(queueFillPercentage()*100)) + "%";
    queue_size.lifetime = ros::Duration(0.0);
    queue_size.frame_locked = true;

    visualization_msgs::MarkerArray marray;
    marray.markers.push_back(queue_size);
    marker_publisher_.publish(marray);
  }
}

void
SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  // Check whether we know about this laser yet
  karto::LaserRangeFinder* laser = getLaser(scan);
  if(!laser)
  {
    ROS_WARN("Failed to create laser device for %s; discarding scan",
	     scan->header.frame_id.c_str());
    return;
  }

  // Convert the laserscan into a Karto object
  karto::LocalizedRangeScan* range_scan = convertScan(laser, scan);
  if (!range_scan)
  {
    return;
  }

  // Check if we have moved enough since the last scan to queue this one
  // Karto performs a similar (hopefully 'identical') check during the
  // Process() call. However, we perform this check here to prevent adding
  // scans to the cache unless Karto will actually use them. Otherwise we
  // will need a very large cache but most entries would just get throw away.
  if (hasMovedEnough(range_scan))
  {
    // Print the debug info
    karto::Pose2 odom_pose = range_scan->GetOdometricPose();
    ROS_DEBUG("added scan at pose: %.3f %.3f %.3f", 
              odom_pose.GetX(),
              odom_pose.GetY(),
              odom_pose.GetHeading());
    // Tag this scan as the most recent scan
    first_scan_received_ = true;
    last_scan_time_ = range_scan->GetTime();
    last_scan_pose_ = range_scan->GetOdometricPose();
    // Push the laserscan into the processing queue
    {
      boost::mutex::scoped_lock lock(scan_queue_mutex_);
      scan_queue_.push_back(range_scan);
      // Pause navigation if the scan queue gets too full
      if (pause_on_full_queue_ && !isPaused() && queueFillPercentage() > pause_navigation_percentage_)
      {
        pauseNavigation();
      }
    }
    // Notify the optimization thread that data is available
    scan_queue_data_available_.notify_one();
  }
}

void
SlamKarto::mapLoop(double map_update_interval)
{
  // Initialize the map information
  map_.map.info.resolution = resolution_;
  map_.map.info.origin.position.x = 0.0;
  map_.map.info.origin.position.y = 0.0;
  map_.map.info.origin.position.z = 0.0;
  map_.map.info.origin.orientation.x = 0.0;
  map_.map.info.origin.orientation.y = 0.0;
  map_.map.info.origin.orientation.z = 0.0;
  map_.map.info.origin.orientation.w = 1.0;

  // If the map update interval is set to zero, never build a map
  if (map_update_interval <= 0)
    return;

  // Configure a rate loop for regenerating the map
  ros::WallRate r(1.0 / map_update_interval);
  while (ros::ok())
  {
    updateMap();
    publishGraphVisualization();
    publishQueueVisualization();
    r.sleep();
  }
}

bool
SlamKarto::updateMap()
{
  // Copy the laserscans locally to minimize the time when karto must be locked
  karto::LocalizedRangeScanVector scans;
  {
    boost::mutex::scoped_lock lock(mapper_mutex_);
    scans = mapper_->GetAllProcessedScans();
  }

  // Build a map from the laserscans
  // If the aligned->map frame is available, build the map in the aligned frame
  // If not, build the map in the normal map frame
  karto::OccupancyGrid* occ_grid;
  std::string output_frame;
  if (tf_.canTransform(map_frame_, aligned_frame_, ros::Time()))
  {
    // Lookup the map->aligned transform
    tf::StampedTransform transform;
    tf_.lookupTransform(aligned_frame_, map_frame_, ros::Time(), transform);
    double yaw = tf::getYaw(transform.getRotation());

    // Make a deep copy of all karto laserscans, transforming the corrected pose into the aligned frame
    karto::LocalizedRangeScanVector transformed_scans;
    transformed_scans.reserve(scans.size());
    karto::Transform map_to_aligned_transform(karto::Pose2(0, 0, yaw));
    for (size_t i = 0; i < scans.size(); ++i)
    {
      const karto::LocalizedRangeScan* pScan = scans.at(i);
      karto::LocalizedRangeScan* transformed_scan = new karto::LocalizedRangeScan(
        pScan->GetSensorName(), pScan->GetRangeReadingsVector());
      transformed_scan->SetOdometricPose(pScan->GetOdometricPose());
      transformed_scan->SetCorrectedPose(map_to_aligned_transform.TransformPose(pScan->GetCorrectedPose()));
      transformed_scans.push_back(transformed_scan);
    }

    // Build the map in the aligned frame
    output_frame = aligned_frame_;
    occ_grid = karto::OccupancyGrid::CreateFromScans(transformed_scans, resolution_);

    // Delete the transformed scans
    for (size_t i = 0; i < transformed_scans.size(); ++i)
    {
      delete transformed_scans.at(i);
    }
    transformed_scans.clear();
  }
  else
  {
    // Build the map in the standard map frame
    output_frame = map_frame_;
    occ_grid = karto::OccupancyGrid::CreateFromScans(scans, resolution_);
  }

  // Abort if no map was generated
  if (occ_grid == NULL)
  {
    return false;
  } 

  // Update the map_ member variable with the newly generated map
  {
    boost::mutex::scoped_lock lock(map_mutex_);

    // Set the header information
    map_.map.header.stamp = ros::Time::now();
    map_.map.header.frame_id = output_frame;

    // Translate to ROS format
    kt_int32s width = occ_grid->GetWidth();
    kt_int32s height = occ_grid->GetHeight();
    karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

    if (map_.map.info.width != (unsigned int) width ||
        map_.map.info.height != (unsigned int) height ||
        map_.map.info.origin.position.x != offset.GetX() ||
        map_.map.info.origin.position.y != offset.GetY())
    {
      map_.map.info.origin.position.x = offset.GetX();
      map_.map.info.origin.position.y = offset.GetY();
      map_.map.info.width = width;
      map_.map.info.height = height;
      map_.map.data.resize(map_.map.info.width * map_.map.info.height);
    }

    for (kt_int32s y = 0; y < height; y++)
    {
      for (kt_int32s x = 0; x < width; x++)
      {
        // Getting the value at position x,y
        kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));
        size_t index = map_.map.info.width * y + x;
        switch (value)
        {
          case karto::GridStates_Unknown:
            map_.map.data[index] = -1;
            break;
          case karto::GridStates_Occupied:
            map_.map.data[index] = 100;
            break;
          case karto::GridStates_Free:
            map_.map.data[index] = 0;
            break;
          default:
            ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
            break;
        }
      }
    }
  }

  // Delete the temporary Karto map object
  delete occ_grid;

  // Publish the map
  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

  // A new map was generated
  got_map_ = true;
  ROS_DEBUG("Updated the map");

  return true;
}

bool SlamKarto::hasMovedEnough(const karto::LocalizedRangeScan* scan)
{
  // If we haven't received a laserscan yet, keep this one.
  if (!first_scan_received_)
  {
    return true;
  }

  // Extract the 2d pose from the current scan
  karto::Pose2 current_laser_pose = scan->GetOdometricPose();

  // Check rotation
  double delta_heading = karto::math::NormalizeAngle(current_laser_pose.GetHeading() - last_scan_pose_.GetHeading());
  if (std::fabs(delta_heading) >= mapper_->getParamMinimumTravelHeading())
  {
    return true;
  }

  // Check distance traveled
  double squared_delta_distance = last_scan_pose_.GetPosition().SquaredDistance(current_laser_pose.GetPosition());
  if (squared_delta_distance >= karto::math::Square(mapper_->getParamMinimumTravelDistance()))
  {
    return true;
  }

  return false;
}

karto::LocalizedRangeScan*
SlamKarto::convertScan(karto::LaserRangeFinder* laser,
  const sensor_msgs::LaserScan::ConstPtr& scan)
{
  karto::Pose2 karto_pose;
  if (!getOdomPose(karto_pose, scan->header.stamp))
     return NULL;
  
  // Create a vector of doubles for karto
  std::vector<kt_double> readings;

  if (lasers_inverted_[scan->header.frame_id]) {
    for(std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
      it != scan->ranges.rend();
      ++it)
    {
      readings.push_back(*it);
    }
  } else {
    for(std::vector<float>::const_iterator it = scan->ranges.begin();
      it != scan->ranges.end();
      ++it)
    {
      readings.push_back(*it);
    }
  }
  
  // create localized range scan
  karto::LocalizedRangeScan* range_scan = 
    new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetTime(scan->header.stamp.toSec());
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  return range_scan;
}

bool 
SlamKarto::mapCallback(nav_msgs::GetMap::Request  &req,
                       nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock lock(map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void
SlamKarto::pauseNavigation()
{
  // Publish a pause navigation message
  if (pause_publisher_.getNumSubscribers() > 0)
  {
    ROS_DEBUG_STREAM("Publishing pause navigation message...");
    std_msgs::Bool msg;
    msg.data = true;
    pause_publisher_.publish(msg);
    is_paused_ = true;
  }

  // Call the pause navigation service
  if (pause_service_client_.exists())
  {
    ROS_DEBUG_STREAM("Calling pause navigation service...");
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (pause_service_client_.call(srv) && srv.response.success)
    {
      is_paused_ = true;
    }
  }
}

void
SlamKarto::resumeNavigation()
{
  // Publish a resume navigation message
  if (pause_publisher_.getNumSubscribers() > 0)
  {
    ROS_DEBUG_STREAM("Publishing resume navigation message...");
    std_msgs::Bool msg;
    msg.data = false;
    pause_publisher_.publish(msg);
    is_paused_ = false;
  }

  // Call the resume navigation service
  if (pause_service_client_.exists())
  {
    ROS_DEBUG_STREAM("Calling resume navigation service...");
    std_srvs::SetBool srv;
    srv.request.data = false;
    if (pause_service_client_.call(srv) && srv.response.success)
    {
      is_paused_ = false;
    }
  }
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_karto");

  SlamKarto kn;

  ros::spin();

  return 0;
}

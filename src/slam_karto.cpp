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
#include "tf/transform_datatypes.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <locus_msgs/GetGraph.h>
#include <locus_msgs/GraphStamped.h>
#include <locus_msgs/GraphUpdate.h>
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include "open_karto/Mapper.h"

#include <slam_karto/SetMapTransform.h>
#include <slam_karto/graph_update.h>
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
     * @brief Callback for service that send the complete Graph to the client
     */
    bool graphCallback(locus_msgs::GetGraph::Request  &req,
                       locus_msgs::GetGraph::Response &res);

    /**
     * @brief Callback that stores the desired map->local transform, used during the map building step.
     */
    bool setMapTransformCallback(slam_karto::SetMapTransform::Request& req, slam_karto::SetMapTransform::Response& rep);

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
     * @brief Create a Path message from the scan data
     */
    nav_msgs::Path createPath(
      const karto::LocalizedRangeScanVector& scans,
      const std::string& frame,
      const ros::Time& stamp);

    /**
     * @brief Copy the scans from the mapper and build a new map.
     *
     * Note that this function blocks, waiting for access to the mapper's set of laserscans.
     */
    bool updateMap();

    /**
     * @brief Transform localized range scans and sync solver
     */
    void transformMap(const karto::Pose2& transform);

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
    ros::Time last_transform_time_;  //!< The timestamp of the last published map->odom transform
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    ros::Publisher sst_;
    ros::Publisher graph_publisher_;  //!< Structure graph representation of the Karto SLAM graph
    ros::Publisher graph_updates_publisher_;  //!< Publish the recent changes to the SLAM graph
    ros::Publisher slam_graph_visualization_publisher_;  //!< Visualization of the Karto SLAM graph
    ros::Publisher scan_queue_visualization_publisher_;  //!< Visualization of the percent fill of the laserscan queue
    ros::Publisher map_path_publisher_;  //!< Publish the entire optimized path in the map frame

    ros::Publisher sstm_;
    ros::ServiceServer ss_;
    ros::ServiceServer graph_server_;  //!< Service server for sending the full graph to clients
    ros::ServiceServer map_transform_service_;  //!< Service server for receiving map->map_local transforms
    ros::Publisher pause_publisher_;  //!< Topic that publishes requests to pause/unpause navigation
    ros::ServiceClient pause_service_client_;  //!< Service client that requests to pause/unpause navigation
    ros::WallTimer queue_visualization_timer_;  //!< Timer used to publish the scan queue length visualization

    // The map that will be published / send to service callers
    nav_msgs::GetMap::Response map_;
    locus_msgs::GraphStamped graph_msg_;
    locus_msgs::GraphStamped graph_update_reference_;

    // Storage for ROS parameters
    std::string odom_frame_;
    std::string map_frame_;  //!< The map will be constructed and published in the map frame
    std::string base_frame_;
    slam_karto::SetMapTransform::Request map_requested_transform_;  //!< The last received map transform request
    bool map_requested_transform_dirty_;  //!< Flag indicating a new request was received
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
    bool map_dirty_;  //!< Flag indicating the map needs to be regenerated
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
    int spa_method_;

    // Laserscan queue for to-be-processed scans
    boost::circular_buffer<karto::LocalizedRangeScan*> scan_queue_;  //!< Fixed-sized buffer for storing range scans
                                                                     //!< that have not been processed by the mapper yet
    boost::condition_variable scan_queue_data_available_;  //!< Variable used to synchronize the producer and consumer
                                                           //!< threads without a busy-wait queue size check
    boost::mutex scan_queue_mutex_;  //!< Mutex lock for queue operations (push, pop, front)
};

namespace
{
// Verify the correlation search space dimension is an even multiple of the resolution

/**
 * @brief Force a dimension (width, height, etc) to be an even (or odd) multiple of a grid resolution
 *
 * @param[in] dimension - The starting dimension
 * @param[in] resolution - The grid resolution
 * @param[in] should_be_even - True if the result should be an even multiple, False if it should be an odd multiple
 * @return The updated dimension that meets the desired properties
 */
double forceEvenOrOdd(const double dimension, const double resolution, const bool should_be_even)
{
  double cells = dimension / resolution;
  double cells_rounded = std::round(dimension / resolution);
  bool exact_multiple = (std::abs(cells - cells_rounded) < 1.0e-6);
  int expected_remainder = should_be_even ? 0 : 1;
  bool is_expected_remainder = (static_cast<int>(cells_rounded) % 2 == expected_remainder);
  if (exact_multiple && is_expected_remainder)
  {
    return dimension;
  }
  else
  {
    if (!is_expected_remainder)
    {
      cells_rounded += 1.0;
    }
    return resolution * cells_rounded;
  }
};
}

SlamKarto::SlamKarto() :
        tf_(ros::Duration(60.0)),
        map_requested_transform_dirty_(false),
        pause_on_loop_closure_(false),
        loop_closure_pauser_(NULL),
        got_map_(false),
        map_dirty_(false),
        laser_count_(0),
        transform_thread_(NULL),
        map_thread_(NULL),
        optimization_thread_(NULL),
        marker_count_(0),
        is_paused_(false),
        first_scan_received_(false),
        last_scan_time_(0.0),
        last_scan_pose_(0, 0, 0),
        scan_queue_(1)
{
  map_to_odom_.setIdentity();
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
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
  private_nh_.param("resume_navigation_percentage", resume_navigation_percentage_, 0.10);
  pause_navigation_percentage_ = boost::algorithm::clamp(pause_navigation_percentage_, 0.0, 1.0);
  resume_navigation_percentage_ = boost::algorithm::clamp(resume_navigation_percentage_,
    0.0, pause_navigation_percentage_);
  int scan_queue_length;
  if (private_nh_.getParam("scan_queue_length", scan_queue_length))
  {
    if (scan_queue_length > 0)
    {
      scan_queue_.set_capacity(scan_queue_length);
      queue_visualization_timer_ = node_.createWallTimer(ros::WallDuration(1.0),
        boost::bind(&SlamKarto::publishQueueVisualization, this));
    }
    else
    {
      ROS_WARN_STREAM("Parameter scan_queue_length must be greater than zero. Ignoring.");
    }
  }

  // Set up advertisements and subscriptions
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  map_path_publisher_ = node_.advertise<nav_msgs::Path>("map_path", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamKarto::mapCallback, this);
  graph_server_ = node_.advertiseService("get_graph", &SlamKarto::graphCallback, this);
  map_transform_service_ = node_.advertiseService("set_map_transform", &SlamKarto::setMapTransformCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamKarto::laserCallback, this, _1));
  graph_publisher_ = node_.advertise<locus_msgs::GraphStamped>("graph", 1);
  graph_updates_publisher_ = node_.advertise<locus_msgs::GraphUpdate>("graph_updates", 1);
  slam_graph_visualization_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("slam_graph", 1);
  scan_queue_visualization_publisher_ = node_.advertise<visualization_msgs::Marker>("scan_queue", 1);
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

  // Verify the correlation search space dimension is an even multiple of the resolution
  double new_correlation_search_space_dimension =
    forceEvenOrOdd(correlation_search_space_dimension, correlation_search_space_resolution, true);
  if (new_correlation_search_space_dimension != correlation_search_space_dimension)
  {
    ROS_WARN_STREAM(
      "The value 'correlation_search_space_dimension' must be an even multiple of "
      "'correlation_search_space_resolution'. Adjusting the value of 'correlation_search_space_dimension' from "
      << correlation_search_space_dimension << " to " << new_correlation_search_space_dimension);
    correlation_search_space_dimension = new_correlation_search_space_dimension;
  }

  double correlation_search_space_smear_deviation;
  if(private_nh_.getParam("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Verify the correlation search smear deviation is an odd multiple of the resolution
  double new_correlation_search_space_smear_deviation =
    forceEvenOrOdd(correlation_search_space_smear_deviation, correlation_search_space_resolution, false);
  if (new_correlation_search_space_smear_deviation != correlation_search_space_smear_deviation)
  {
    ROS_WARN_STREAM(
      "The value 'correlation_search_space_smear_deviation' must be an odd multiple of "
      "'correlation_search_space_resolution'. Adjusting the value of 'correlation_search_space_smear_deviation' from "
      << correlation_search_space_smear_deviation << " to " << new_correlation_search_space_smear_deviation);
    correlation_search_space_smear_deviation = new_correlation_search_space_smear_deviation;
  }

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter Server
  double loop_search_space_dimension;
  if(private_nh_.getParam("loop_search_space_dimension", loop_search_space_dimension))
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution;
  if(private_nh_.getParam("loop_search_space_resolution", loop_search_space_resolution))
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  // Verify the loop search space dimension is an even multiple of the resolution
  double new_loop_search_space_dimension =
    forceEvenOrOdd(loop_search_space_dimension, loop_search_space_resolution, true);
  if (new_loop_search_space_dimension != loop_search_space_dimension)
  {
    ROS_WARN_STREAM(
      "The value 'loop_search_space_dimension' must be an even multiple of 'loop_search_space_resolution'. Adjusting "
      "the value of 'loop_search_space_dimension' from "
      << loop_search_space_dimension << " to " << new_loop_search_space_dimension);
    loop_search_space_dimension = new_loop_search_space_dimension;
  }

  double loop_search_space_smear_deviation;
  if(private_nh_.getParam("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
    mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Verify the loop search smear deviation is an odd multiple of the resolution
  double new_loop_search_space_smear_deviation =
    forceEvenOrOdd(loop_search_space_smear_deviation, loop_search_space_resolution, false);
  if (new_loop_search_space_smear_deviation != loop_search_space_smear_deviation)
  {
    ROS_WARN_STREAM(
      "The value 'loop_search_space_smear_deviation' must be an odd multiple of 'loop_search_space_resolution'. "
      "Adjusting the value of 'loop_search_space_smear_deviation' from "
      << loop_search_space_smear_deviation << " to " << new_loop_search_space_smear_deviation);
    loop_search_space_smear_deviation = new_loop_search_space_smear_deviation;
  }

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
  spa_method_ = SBA_SPARSE_CHOLESKY;

  if(private_nh_.getParam("spa_method", spa_method_string))
  {
    if(spa_method_string == "dense_cholesky")
      spa_method_ = SBA_DENSE_CHOLESKY;
    else if(spa_method_string == "gradient")
      spa_method_ = SBA_GRADIENT;
    else if(spa_method_string == "block_jacobian_pcg")
      spa_method_ = SBA_BLOCK_JACOBIAN_PCG;
    else if(spa_method_string != "sparse_cholesky")
      ROS_WARN_STREAM("\"" << spa_method_string << "\" is an invalid spa_parameter value. Valid values are "
          "\"sparse_cholesky,\" \"dense_cholesky,\" \"gradient,\" and \"block_jacobian_pcg.\" "
          "Assuming sparse_cholesky.");
  }

  solver_->SetSpaMethod(spa_method_);
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
  if (transform_thread_)
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
    // Wait for data to arrive.
    karto::LocalizedRangeScan* range_scan;
    {
      // The while-loop guards against spurious wakeups.
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
      range_scan = scan_queue_.front();
      scan_queue_.pop_front();
    }
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
      // Mark the map as needing to be updated
      {
        boost::mutex::scoped_lock lock(map_mutex_);
        map_dirty_ = true;
      }
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
  ros::Time transform_time = ros::Time::now();
  if (transform_time != last_transform_time_)
  {
    auto map_to_odom = geometry_msgs::TransformStamped();
    map_to_odom.header.stamp = transform_time;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    tf::transformTFToMsg(map_to_odom_,map_to_odom.transform);
    static_broadcaster_.sendTransform(map_to_odom);
    last_transform_time_ = transform_time;
  }
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
    catch(const tf::TransformException& e)
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
    catch (const tf::TransformException& e)
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

    // Karto uses some directionality checks to determine laserscan validity.
    // https://github.com/ros-perception/open_karto/blob/melodic-devel/src/Mapper.cpp#L763-L767
    // This does not play well with lidars that spin clockwise instead of counter-clockwise.
    // If this is a clockwise lidar, reverse the spin direction by:
    // * Swapping the min and max angles
    // * Inverting the angle increment
    // * Marking the lidar as inverted, which reverses the order of the range readings
    if (scan->angle_increment > 0)
    {
      laser->SetMinimumAngle(scan->angle_min);
      laser->SetMaximumAngle(scan->angle_max);
      laser->SetAngularResolution(scan->angle_increment);
    }
    else
    {
      laser->SetMinimumAngle(scan->angle_max);
      laser->SetMaximumAngle(scan->angle_min);
      laser->SetAngularResolution(-scan->angle_increment);
      lasers_inverted_[scan->header.frame_id] = !inverse;
    }
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
  catch(const tf::TransformException& e)
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
  if (slam_graph_visualization_publisher_.getNumSubscribers() > 0)
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
    // Fix rviz warning about quaternions
    nodes.pose.orientation.w = 1.0;
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
    // Fix rviz warning about quaternions
    edges.pose.orientation.w = 1.0;
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

    slam_graph_visualization_publisher_.publish(marray);
  }
}

void
SlamKarto::publishQueueVisualization()
{
  // Publish queue status
  if (scan_queue_.capacity() > 1 && scan_queue_visualization_publisher_.getNumSubscribers() > 0)
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

    scan_queue_visualization_publisher_.publish(queue_size);
  }
}

void
SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  auto is_scan_valid = [](const sensor_msgs::LaserScan& scan)
  {
    auto expected_scans = static_cast<int>(std::round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1;
    return (std::abs(scan.angle_increment) < 0.1) &&
           (std::abs(expected_scans - static_cast<int>(scan.ranges.size())) <= 1);
  };
  if (!is_scan_valid(*scan))
  {
    ROS_WARN_STREAM_THROTTLE(
      10.0,
      "Scan at " << scan->header.stamp << " is invalid (range count = " << scan->ranges.size() << "). Skipping scan.");
    return;
  }

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
  else
  {
    delete range_scan;
  }
}

bool
SlamKarto::setMapTransformCallback(
  slam_karto::SetMapTransform::Request& req,
  slam_karto::SetMapTransform::Response& res)
{
  boost::mutex::scoped_lock lock(map_mutex_);
  map_requested_transform_ = req;
  map_requested_transform_dirty_= true;

  res.success = true;
  return true;
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
    r.sleep();
  }
}

nav_msgs::Path
SlamKarto::createPath(const karto::LocalizedRangeScanVector& scans, const std::string& frame, const ros::Time& stamp)
{
  nav_msgs::Path path_msg;
  path_msg.header.stamp = stamp;
  path_msg.header.frame_id = frame;
  for (size_t i = 0; i < scans.size(); ++i)
  {
    const karto::Pose2& pose_2d = scans[i]->GetCorrectedPose();
    geometry_msgs::PoseStamped pose_3d;
    pose_3d.header.stamp.fromSec(scans[i]->GetTime());
    pose_3d.header.frame_id = frame;
    pose_3d.pose.position.x = pose_2d.GetX();
    pose_3d.pose.position.y = pose_2d.GetY();
    pose_3d.pose.position.z = 0.0;
    pose_3d.pose.orientation = tf::createQuaternionMsgFromYaw(pose_2d.GetHeading());
    path_msg.poses.push_back(pose_3d);
  }
  return path_msg;
}

void SlamKarto::transformMap(const karto::Pose2& transform)
{
  boost::mutex::scoped_lock lock(mapper_mutex_);
  karto::LocalizedRangeScanVector source_scans = mapper_->GetAllProcessedScans();
  karto::Transform map_to_odom_transform(transform);
  for (auto& scan : source_scans)
  {
    scan->SetCorrectedPose(map_to_odom_transform.TransformPose(scan->GetCorrectedPose()));
  }

  updateMapToOdomTransform(*source_scans.rbegin());
  // Sync solver
  if (solver_)
  {
    // There is no easy way to clear constraints, rebuild solver
    delete solver_;
    solver_ = new SpaSolver();
    solver_->SetSpaMethod(spa_method_);
    mapper_->SetScanSolver(solver_);
    // add the nodes to the optimizer
    auto mapper_vertices = mapper_->GetGraph()->GetVertices();
    for (auto& [sensor, vertices] : mapper_vertices)
    {
      for (auto& vertex : vertices)
      {
        if (nullptr != vertex)
        {
          solver_->AddNode(vertex);
        }
      }
    }
    // add constraints to the optimizer
    auto edges = mapper_->GetGraph()->GetEdges();
    for (auto& edge : edges)
    {
      if (edge != nullptr)
      {
        solver_->AddConstraint(edge);
      }
    }
  }
}

bool
SlamKarto::updateMap()
{
  bool map_dirty;
  bool map_requested_transform_dirty;
  slam_karto::SetMapTransform::Request map_requested_transform;
  {
    boost::mutex::scoped_lock lock(map_mutex_);
    map_dirty = map_dirty_;
    map_requested_transform_dirty = map_requested_transform_dirty_;
    map_requested_transform = map_requested_transform_;
    map_dirty_ = false;
    map_requested_transform_dirty_ = false;
  }

  if (!map_dirty && !map_requested_transform_dirty)
  {
    // Everything is up to date.
    return false;
  }

  // Grab the current time to use in all of the messages
  ros::Time current_time = ros::Time::now();

  if (map_requested_transform_dirty)
  {
    transformMap(karto::Pose2(
      map_requested_transform.transform.translation.x,
      map_requested_transform.transform.translation.y,
      tf::getYaw(map_requested_transform.transform.rotation)));
  }

  // Copy the laserscans locally to minimize the time when karto must be locked
  karto::LocalizedRangeScanVector scans;
  auto graph_msg = locus_msgs::GraphStamped();
  {
    boost::mutex::scoped_lock lock(mapper_mutex_);
    karto::LocalizedRangeScanVector source_scans = mapper_->GetAllProcessedScans();
    scans.reserve(source_scans.size());
    for (size_t i = 0; i < source_scans.size(); ++i)
    {
      const karto::LocalizedRangeScan* source_scan = source_scans.at(i);
      karto::LocalizedRangeScan* scan = new karto::LocalizedRangeScan(
          source_scan->GetSensorName(), source_scan->GetRangeReadingsVector());
      scan->SetOdometricPose(source_scan->GetOdometricPose());
      scan->SetCorrectedPose(source_scan->GetCorrectedPose());
      scan->SetTime(source_scan->GetTime());
      scans.push_back(scan);
    }

    // Populate the graph
    graph_msg.header.stamp = current_time;
    graph_msg.header.frame_id = map_frame_;
    for (const auto& [name, vertices] : mapper_->GetGraph()->GetVertices())
    {
      for (const auto& vertex : vertices)
      {
        auto node_msg = locus_msgs::Node();
        node_msg.id = vertex->GetObject()->GetUniqueId();
        node_msg.position.x = vertex->GetObject()->GetCorrectedPose().GetX();
        node_msg.position.y = vertex->GetObject()->GetCorrectedPose().GetY();
        graph_msg.graph.nodes.push_back(node_msg);
      }
    }
    for (const auto& edge : mapper_->GetGraph()->GetEdges())
    {
      auto edge_msg = locus_msgs::Edge();
      edge_msg.node_ids[0] = edge->GetSource()->GetObject()->GetUniqueId();
      edge_msg.node_ids[1] = edge->GetTarget()->GetObject()->GetUniqueId();
      graph_msg.graph.edges.push_back(edge_msg);
    }
    // Force the graph structure to be sorted. This will allow easier comparisons later.
    std::sort(
      graph_msg.graph.nodes.begin(),
      graph_msg.graph.nodes.end(),
      slam_karto::nodeComparison);
    std::sort(
      graph_msg.graph.edges.begin(),
      graph_msg.graph.edges.end(),
      slam_karto::edgeComparison);
  }

  // Build a map from the laserscans
  karto::OccupancyGrid* occ_grid = karto::OccupancyGrid::CreateFromScans(scans, resolution_);

  // If requested to zero the origin, correct all of the scans with the map offset
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();
  if (map_requested_transform_dirty && map_requested_transform.zero_origin)
  {
    karto::Pose2 offset_pose(-offset.GetX(), -offset.GetY(), 0.0);
    transformMap(offset_pose);
    karto::Transform offset_transform(offset_pose);
    for (size_t i = 0; i < scans.size(); ++i)
    {
      karto::LocalizedRangeScan* scan = scans.at(i);
      scan->SetCorrectedPose(offset_transform.TransformPose(scan->GetCorrectedPose()));
    }
    offset.SetX(0.0);
    offset.SetY(0.0);
  }

  // Create the path message. The path message is published in the "map" frame
  map_path_publisher_.publish(createPath(scans, map_frame_, current_time));

  // Delete the copied scans
  for (size_t i = 0; i < scans.size(); ++i)
  {
    delete scans.at(i);
  }
  scans.clear();

  // Abort if no map was generated
  if (occ_grid == NULL)
  {
    return false;
  }

  // Update the map_ member variable with the newly generated map
  {
    boost::mutex::scoped_lock lock(map_mutex_);

    // Set the header information
    map_.map.header.stamp = current_time;
    map_.map.header.frame_id = map_frame_;

    // Reallocate memory if the map changes size
    kt_int32s width = occ_grid->GetWidth();
    kt_int32s height = occ_grid->GetHeight();
    if (map_.map.info.width != (unsigned int) width ||
        map_.map.info.height != (unsigned int) height)
    {
      map_.map.data.resize(width * height);
    }

    // Translate to ROS format
    map_.map.info.map_load_time = current_time;
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;

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
    got_map_ = true;

    // Publish the map
    sst_.publish(map_.map);
    sstm_.publish(map_.map.info);

    // Publish the new graph
    std::swap(graph_msg_, graph_msg);
    graph_publisher_.publish(graph_msg_);

    // Publish just the major changes to the graph
    // The graph_update_reference_ accumulates all of the previously published changes
    // By comparing the accumulated updates to the full graph, we can find all changes, even if they happen slowly.
    auto graph_update = slam_karto::computeGraphChanges(graph_msg_, graph_update_reference_);
    slam_karto::applyGraphUpdates(graph_update_reference_, graph_update);
    graph_updates_publisher_.publish(graph_update);
  }

  // Delete the temporary Karto map object
  delete occ_grid;

  // A new map was generated
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
  if (std::fabs(delta_heading) >= karto::math::DegreesToRadians(mapper_->getParamMinimumTravelHeading()))
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

bool
SlamKarto::graphCallback(locus_msgs::GetGraph::Request  &req,
                         locus_msgs::GetGraph::Response &res)
{
  boost::mutex::scoped_lock lock(map_mutex_);
  res.graph = graph_msg_;
  res.success = true;
  return true;
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

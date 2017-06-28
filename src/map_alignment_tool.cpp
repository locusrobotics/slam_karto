/***************************************************************************
 * Copyright (C) 2017 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#include <slam_karto/map_alignment_tool.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>

namespace slam_karto
{

static const std::string endpoint1_name = "endpoint1";
static const std::string endpoint2_name = "endpoint2";
static const std::string connector_name = "connector";

MapAlignmentTool::MapAlignmentTool(const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle) :
  node_handle_(node_handle),
  interactive_marker_server_("map_alignment_tool"),
  aligned_frame_("map_aligned"),
  map_frame_("map")
{
  // Read the frame names from the parameter server
  private_node_handle.getParam("aligned_frame", aligned_frame_);
  private_node_handle.getParam("map_frame", map_frame_);
  // Create the map alignment visualization markers
  visualization_msgs::InteractiveMarker endpoint1 = createEndpoint(endpoint1_name);
  visualization_msgs::InteractiveMarker endpoint2 = createEndpoint(endpoint2_name);
  visualization_msgs::InteractiveMarker connector = createConnector(connector_name);
  // Set the default positions of the markers
  endpoint1.pose.position.x = -1.0;
  endpoint2.pose.position.x = +1.0;
  connector.controls.at(0).markers.at(0).points.at(0) = endpoint1.pose.position;
  connector.controls.at(0).markers.at(0).points.at(1) = endpoint2.pose.position;
  // Add the markers to the interactive marker server
  interactive_marker_server_.insert(endpoint1, boost::bind(&MapAlignmentTool::endpointCallback, this, _1));
  interactive_marker_server_.insert(endpoint2, boost::bind(&MapAlignmentTool::endpointCallback, this, _1));
  interactive_marker_server_.insert(connector);
  // Define the menu handler
  menu_handler_.insert("Align Map", boost::bind(&MapAlignmentTool::alignMapCallback, this, _1));
  menu_handler_.apply(interactive_marker_server_, connector_name);
  // And finally apply all of the changes so the clients can see them
  interactive_marker_server_.applyChanges();
}

MapAlignmentTool::~MapAlignmentTool()
{
}

visualization_msgs::InteractiveMarker MapAlignmentTool::createEndpoint(const std::string& name) const
{
  // Create alignment markers
  visualization_msgs::InteractiveMarker endpoint;
  endpoint.header.frame_id = map_frame_;
  endpoint.name = name;
  endpoint.pose.position.x = 0;
  endpoint.pose.position.y = 0;
  endpoint.pose.position.z = 0;
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 0.707106781;
  control.orientation.x = 0;
  control.orientation.y = -0.707106781;
  control.orientation.z = 0;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  visualization_msgs::Marker visual;
  visual.type = visualization_msgs::Marker::SPHERE;
  visual.scale.x = 0.20;
  visual.scale.y = 0.20;
  visual.scale.z = 0.20;
  visual.color.r = 0.0;
  visual.color.g = 0.0;
  visual.color.b = 1.0;
  visual.color.a = 0.7;
  control.markers.push_back(visual);
  endpoint.controls.push_back(control);

  return endpoint;
}

visualization_msgs::InteractiveMarker MapAlignmentTool::createConnector(const std::string& name) const
{
  visualization_msgs::InteractiveMarker connector;
  connector.header.frame_id = map_frame_;
  connector.name = name;
  connector.pose.position.x = 0;
  connector.pose.position.y = 0;
  connector.pose.position.z = 0;
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  visualization_msgs::Marker visual;
  visual.type = visualization_msgs::Marker::ARROW;
  visual.scale.x = 0.1;
  visual.scale.y = 0.2;
  visual.scale.z = 0.2;
  visual.color.r = 1.0;
  visual.color.g = 0.9;
  visual.color.b = 0.4;
  visual.color.a = 0.5;
  visual.points.push_back(geometry_msgs::Point());
  visual.points.push_back(geometry_msgs::Point());
  control.markers.push_back(visual);
  connector.controls.push_back(control);

  return connector;
}

void MapAlignmentTool::endpointCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // Query the two alignment handle markers from the marker server
  visualization_msgs::InteractiveMarker endpoint1;
  visualization_msgs::InteractiveMarker endpoint2;
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE
    && interactive_marker_server_.get(endpoint1_name, endpoint1)
    && interactive_marker_server_.get(endpoint2_name, endpoint2))
  {
    // Update the arrow to connect the two alignment handles together
    visualization_msgs::InteractiveMarker connector = createConnector(connector_name);
    connector.controls.at(0).markers.at(0).points.at(0) = endpoint1.pose.position;
    connector.controls.at(0).markers.at(0).points.at(1) = endpoint2.pose.position;
    // Insert the updated connector marker into the marker server
    interactive_marker_server_.insert(connector);
    menu_handler_.reApply(interactive_marker_server_);
    interactive_marker_server_.applyChanges();
  }
}

void MapAlignmentTool::alignMapCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // Query the two alignment handle markers from the marker server
  visualization_msgs::InteractiveMarker endpoint1;
  visualization_msgs::InteractiveMarker endpoint2;
  if (interactive_marker_server_.get(endpoint1_name, endpoint1)
    && interactive_marker_server_.get(endpoint2_name, endpoint2))
  {
    // Compute yaw
    double yaw = std::atan2(endpoint2.pose.position.y - endpoint1.pose.position.y,
      endpoint2.pose.position.x - endpoint1.pose.position.x);

    // Publish aligned->map frame transformation
    geometry_msgs::TransformStamped aligned_to_map_transform;
    aligned_to_map_transform.header.stamp = ros::Time(0, 0);
    aligned_to_map_transform.header.frame_id = aligned_frame_;
    aligned_to_map_transform.child_frame_id = map_frame_;
    aligned_to_map_transform.transform.translation.x = 0;
    aligned_to_map_transform.transform.translation.y = 0;
    aligned_to_map_transform.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, -yaw);  // send the opposite rotation so the map frame will end up in the requested orientation
    aligned_to_map_transform.transform.rotation.x = q.x();
    aligned_to_map_transform.transform.rotation.y = q.y();
    aligned_to_map_transform.transform.rotation.z = q.z();
    aligned_to_map_transform.transform.rotation.w = q.w();
    static_broadcaster_.sendTransform(aligned_to_map_transform);
  }
}

}  // namespace slam_karto

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_alignment_tool");
  slam_karto::MapAlignmentTool alignment_tool;

  ros::spin();

  return 0;
}

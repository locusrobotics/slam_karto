/***************************************************************************
 * Copyright (C) 2023 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#include <slam_karto/graph_update.h>

#include <locus_cpp/math.h>
#include <locus_msgs/Edge.h>
#include <locus_msgs/GraphStamped.h>
#include <locus_msgs/GraphUpdate.h>
#include <locus_msgs/Node.h>

#include <ros/console.h>

#include <algorithm>
#include <iterator>

namespace slam_karto
{
locus_msgs::GraphUpdate computeGraphChanges(
  const locus_msgs::GraphStamped& current,
  const locus_msgs::GraphStamped& previous,
  const double change_threshold)
{
  auto updates = locus_msgs::GraphUpdate();
  updates.header = current.header;
  // Find nodes in the current graph that do not exist in the previous graph, or have substantially changed position
  // The comparison function first checks that the lhs ID is smaller than the rhs ID. If the IDs are the same, we then
  // check the different in positions. Instead of a standard equality, we allow +/- tolerance around the positions to
  // mean "equal" as well.
  std::set_difference(
    current.graph.nodes.begin(),
    current.graph.nodes.end(),
    previous.graph.nodes.begin(),
    previous.graph.nodes.end(),
    std::back_inserter(updates.node_changes),
    [tolerance = change_threshold](const locus_msgs::Node& lhs, const locus_msgs::Node& rhs)
    {
      return (lhs.id < rhs.id) ||
             (lhs.id == rhs.id && ((lhs.position.x < rhs.position.x - tolerance) ||
             (lhs.position.x < rhs.position.x + tolerance && lhs.position.y < rhs.position.y - tolerance)));
    });

  // Find the edges in the current graph that do not exist in the previous graph
  std::set_difference(
    current.graph.edges.begin(),
    current.graph.edges.end(),
    previous.graph.edges.begin(),
    previous.graph.edges.end(),
    std::back_inserter(updates.edge_changes),
    [](const locus_msgs::Edge& lhs, const locus_msgs::Edge& rhs)
    {
      return (lhs.node_ids[0] < rhs.node_ids[0]) ||
             (lhs.node_ids[0] == rhs.node_ids[0] && lhs.node_ids[1] < rhs.node_ids[1]);
    });

  return updates;
}

}  // namespace slam_karto

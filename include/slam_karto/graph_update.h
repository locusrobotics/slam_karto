/***************************************************************************
 * Copyright (C) 2023 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#ifndef SLAM_KARTO_GRAPH_UPDATE_H
#define SLAM_KARTO_GRAPH_UPDATE_H

#include <locus_msgs/Edge.h>
#include <locus_msgs/GraphStamped.h>
#include <locus_msgs/GraphUpdate.h>
#include <locus_msgs/Node.h>

namespace slam_karto
{
/**
 * @brief Update the input graph with the supplied set of graph updates
 *
 * The input graph and changes are expected to be sorted by node id for both the nodes list and edges list. Calling
 * applyGraphChanges() on unsorted graphs results in undefined behavior.
 *
 * @param[in,out] graph - Graph to update
 * @param[in] changes - A set of graph changes
 */
void applyGraphChanges(locus_msgs::GraphStamped& current, const locus_msgs::GraphUpdate& changes);

/**
 * @brief Compute the changes between two graphs
 *
 * The input graphs are expected to be sorted by node id for both the nodes list and edges list. Calling
 * computeGraphChanges() on unsorted graphs results in undefined behavior.
 *
 * @param[in] current - The new graph
 * @param[in] previous - The original reference graph
 * @param[in] change_threshold - Tolerence for node position changes
 * @return The set of additions and modification present in \p current that do not exist in \p previous
 */
locus_msgs::GraphUpdate computeGraphChanges(
  const locus_msgs::GraphStamped& current,
  const locus_msgs::GraphStamped& previous,
  const double change_threshold = 0.05);

/**
 * @brief Comparison function for edges used as sorting criteria
 *
 * @param[in] lhs - Edge on the left hand side of the comparison function
 * @param[in] rhs - Edge on the right hand side of the comparison function
 * @return True if lhs is less than rhs
 */
bool edgeComparison(const locus_msgs::Edge& lhs, const locus_msgs::Edge& rhs);

/**
 * @brief Comparison function for nodes used as sorting criteria
 *
 * @param[in] lhs - Node on the left hand side of the comparison function
 * @param[in] rhs - Node on the right hand side of the comparison function
 * @return True if lhs is less than rhs
 */
bool nodeComparison(const locus_msgs::Node& lhs, const locus_msgs::Node& rhs);
}  // namespace slam_karto

#endif  // SLAM_KARTO_GRAPH_UPDATE_H

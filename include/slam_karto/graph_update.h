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

}  // namespace slam_karto

#endif  // SLAM_KARTO_GRAPH_UPDATE_H

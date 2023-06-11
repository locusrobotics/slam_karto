/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <locus_msgs/Edge.h>
#include <locus_msgs/GraphStamped.h>
#include <locus_msgs/GraphUpdate.h>
#include <locus_msgs/Node.h>
#include <slam_karto/graph_update.h>

#include <gtest/gtest.h>


TEST(ApplyGraphChanges, EmptyGraph)
{
  // Test that starting with an empty graph works as expected

  auto graph = locus_msgs::GraphStamped();

  auto updates = locus_msgs::GraphUpdate();
  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0f;
  node1.position.y = 2.0f;
  updates.node_changes.push_back(node1);

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1f;
  node2.position.y = 2.2f;
  updates.node_changes.push_back(node2);

  auto edge1 = locus_msgs::Edge();
  edge1.node_ids[0] = 1;
  edge1.node_ids[1] = 2;
  updates.edge_changes.push_back(edge1);

  slam_karto::applyGraphUpdates(graph, updates);

  ASSERT_EQ(graph.graph.nodes.size(), 2u);
  EXPECT_EQ(graph.graph.nodes[0].id, 1u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.x, 1.0f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.y, 2.0f);
  EXPECT_EQ(graph.graph.nodes[1].id, 2u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.x, 1.1f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.y, 2.2f);

  ASSERT_EQ(graph.graph.edges.size(), 1u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[0], 1u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[1], 2u);
}

TEST(ApplyGraphChanges, EmptyUpdates)
{
  // Test that applying an empty change set does not alter the graph

  auto graph = locus_msgs::GraphStamped();

  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0f;
  node1.position.y = 2.0f;
  graph.graph.nodes.push_back(node1);

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1f;
  node2.position.y = 2.2f;
  graph.graph.nodes.push_back(node2);

  auto edge1 = locus_msgs::Edge();
  edge1.node_ids[0] = 1;
  edge1.node_ids[1] = 2;
  graph.graph.edges.push_back(edge1);

  auto updates = locus_msgs::GraphUpdate();

  slam_karto::applyGraphUpdates(graph, updates);

  ASSERT_EQ(graph.graph.nodes.size(), 2u);
  EXPECT_EQ(graph.graph.nodes[0].id, 1u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.x, 1.0f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.y, 2.0f);
  EXPECT_EQ(graph.graph.nodes[1].id, 2u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.x, 1.1f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.y, 2.2f);

  ASSERT_EQ(graph.graph.edges.size(), 1u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[0], 1u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[1], 2u);
}

TEST(ApplyGraphChanges, NewEntries)
{
  // Test that a non-empty graph is updated to contain new nodes and edges

  auto graph = locus_msgs::GraphStamped();

  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0f;
  node1.position.y = 2.0f;
  graph.graph.nodes.push_back(node1);

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1f;
  node2.position.y = 2.2f;
  graph.graph.nodes.push_back(node2);

  auto edge1 = locus_msgs::Edge();
  edge1.node_ids[0] = 1;
  edge1.node_ids[1] = 2;
  graph.graph.edges.push_back(edge1);

  auto updates = locus_msgs::GraphUpdate();

  auto node3 = locus_msgs::Node();
  node3.id = 3;
  node3.position.x = 1.5f;
  node3.position.y = 2.5f;
  updates.node_changes.push_back(node3);

  auto edge2 = locus_msgs::Edge();
  edge2.node_ids[0] = 1;
  edge2.node_ids[1] = 3;
  updates.edge_changes.push_back(edge2);

  auto edge3 = locus_msgs::Edge();
  edge3.node_ids[0] = 2;
  edge3.node_ids[1] = 3;
  updates.edge_changes.push_back(edge3);

  slam_karto::applyGraphUpdates(graph, updates);

  ASSERT_EQ(graph.graph.nodes.size(), 3u);
  EXPECT_EQ(graph.graph.nodes[0].id, 1u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.x, 1.0f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.y, 2.0f);
  EXPECT_EQ(graph.graph.nodes[1].id, 2u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.x, 1.1f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.y, 2.2f);
  EXPECT_EQ(graph.graph.nodes[2].id, 3u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[2].position.x, 1.5f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[2].position.y, 2.5f);

  ASSERT_EQ(graph.graph.edges.size(), 3u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[0], 1u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[1], 2u);
  EXPECT_EQ(graph.graph.edges[1].node_ids[0], 1u);
  EXPECT_EQ(graph.graph.edges[1].node_ids[1], 3u);
  EXPECT_EQ(graph.graph.edges[2].node_ids[0], 2u);
  EXPECT_EQ(graph.graph.edges[2].node_ids[1], 3u);
}

TEST(ApplyGraphChanges, ChangedEntries)
{
  // Test that a non-empty graph can have existing node positions updated

  auto graph = locus_msgs::GraphStamped();

  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0f;
  node1.position.y = 2.0f;
  graph.graph.nodes.push_back(node1);

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1f;
  node2.position.y = 2.2f;
  graph.graph.nodes.push_back(node2);

  auto edge1 = locus_msgs::Edge();
  edge1.node_ids[0] = 1;
  edge1.node_ids[1] = 2;
  graph.graph.edges.push_back(edge1);

  auto updates = locus_msgs::GraphUpdate();

  node1.position.x = 1.05f;
  node1.position.y = 2.05f;
  updates.node_changes.push_back(node1);

  slam_karto::applyGraphUpdates(graph, updates);

  ASSERT_EQ(graph.graph.nodes.size(), 2u);
  EXPECT_EQ(graph.graph.nodes[0].id, 1u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.x, 1.05f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.y, 2.05f);
  EXPECT_EQ(graph.graph.nodes[1].id, 2u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.x, 1.1f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.y, 2.2f);

  ASSERT_EQ(graph.graph.edges.size(), 1u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[0], 1u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[1], 2u);
}

TEST(ApplyGraphChanges, RemainsSorted)
{
  // Verify the new nodes and edges are inserted in the correct location in the graph

  auto graph = locus_msgs::GraphStamped();

  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0f;
  node1.position.y = 2.0f;
  graph.graph.nodes.push_back(node1);

  auto node3 = locus_msgs::Node();
  node3.id = 3;
  node3.position.x = 1.5f;
  node3.position.y = 2.5f;
  graph.graph.nodes.push_back(node3);

  auto edge2 = locus_msgs::Edge();
  edge2.node_ids[0] = 1;
  edge2.node_ids[1] = 3;
  graph.graph.edges.push_back(edge2);

  auto updates = locus_msgs::GraphUpdate();

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1f;
  node2.position.y = 2.2f;
  updates.node_changes.push_back(node2);

  auto edge1 = locus_msgs::Edge();
  edge1.node_ids[0] = 1;
  edge1.node_ids[1] = 2;
  updates.edge_changes.push_back(edge1);

  auto edge3 = locus_msgs::Edge();
  edge3.node_ids[0] = 2;
  edge3.node_ids[1] = 3;
  updates.edge_changes.push_back(edge3);

  slam_karto::applyGraphUpdates(graph, updates);

  ASSERT_EQ(graph.graph.nodes.size(), 3u);
  EXPECT_EQ(graph.graph.nodes[0].id, 1u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.x, 1.0f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[0].position.y, 2.0f);
  EXPECT_EQ(graph.graph.nodes[1].id, 2u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.x, 1.1f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[1].position.y, 2.2f);
  EXPECT_EQ(graph.graph.nodes[2].id, 3u);
  EXPECT_FLOAT_EQ(graph.graph.nodes[2].position.x, 1.5f);
  EXPECT_FLOAT_EQ(graph.graph.nodes[2].position.y, 2.5f);

  ASSERT_EQ(graph.graph.edges.size(), 3u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[0], 1u);
  EXPECT_EQ(graph.graph.edges[0].node_ids[1], 2u);
  EXPECT_EQ(graph.graph.edges[1].node_ids[0], 1u);
  EXPECT_EQ(graph.graph.edges[1].node_ids[1], 3u);
  EXPECT_EQ(graph.graph.edges[2].node_ids[0], 2u);
  EXPECT_EQ(graph.graph.edges[2].node_ids[1], 3u);
}

TEST(ComputeGraphChanges, Empty)
{
  // Test that empty input messages do not cause problems. Output should also be empty.

  auto previous = locus_msgs::GraphStamped();
  auto current = locus_msgs::GraphStamped();

  auto change_threshold = 0.05;
  auto result = slam_karto::computeGraphChanges(current, previous, change_threshold);

  EXPECT_TRUE(result.node_changes.empty());
  EXPECT_TRUE(result.edge_changes.empty());
}

TEST(ComputeGraphChanges, EmptyCurrentGraph)
{
  // Test that a non-empty previous graph and an empty current graph returns an empty set. This is a short coming of
  // the current Graph message format; there is no way to represent a deletion.

  auto previous = locus_msgs::GraphStamped();
  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0;
  node1.position.y = 2.0;
  previous.graph.nodes.push_back(node1);

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1;
  node2.position.y = 2.2;
  previous.graph.nodes.push_back(node2);

  auto edge1 = locus_msgs::Edge();
  edge1.node_ids[0] = 1;
  edge1.node_ids[1] = 2;
  previous.graph.edges.push_back(edge1);

  auto current = locus_msgs::GraphStamped();

  auto change_threshold = 0.05;
  auto result = slam_karto::computeGraphChanges(current, previous, change_threshold);

  EXPECT_TRUE(result.node_changes.empty());
  EXPECT_TRUE(result.edge_changes.empty());
}

TEST(ComputeGraphChanges, EmptyPreviousGraph)
{
  // Test that an empty previous graph and a non-empty current graph returns all nodes and edges in the change set.

  auto previous = locus_msgs::GraphStamped();

  auto current = locus_msgs::GraphStamped();
  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0f;
  node1.position.y = 2.0f;
  current.graph.nodes.push_back(node1);

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1f;
  node2.position.y = 2.2f;
  current.graph.nodes.push_back(node2);

  auto edge1 = locus_msgs::Edge();
  edge1.node_ids[0] = 1;
  edge1.node_ids[1] = 2;
  current.graph.edges.push_back(edge1);

  auto change_threshold = 0.05;
  auto result = slam_karto::computeGraphChanges(current, previous, change_threshold);

  ASSERT_EQ(result.node_changes.size(), 2u);
  EXPECT_EQ(result.node_changes[0].id, 1u);
  EXPECT_FLOAT_EQ(result.node_changes[0].position.x, 1.0f);
  EXPECT_FLOAT_EQ(result.node_changes[0].position.y, 2.0f);
  EXPECT_EQ(result.node_changes[1].id, 2u);
  EXPECT_FLOAT_EQ(result.node_changes[1].position.x, 1.1f);
  EXPECT_FLOAT_EQ(result.node_changes[1].position.y, 2.2f);

  ASSERT_EQ(result.edge_changes.size(), 1u);
  EXPECT_EQ(result.edge_changes[0].node_ids[0], 1u);
  EXPECT_EQ(result.edge_changes[0].node_ids[1], 2u);
}

TEST(ComputeGraphChanges, NewEntries)
{
  // Include some new entries along with the original entries in the current graph. The output should only contain
  // the new entries.

  auto previous = locus_msgs::GraphStamped();
  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0;
  node1.position.y = 2.0;
  previous.graph.nodes.push_back(node1);

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1;
  node2.position.y = 2.2;
  previous.graph.nodes.push_back(node2);

  auto edge1 = locus_msgs::Edge();
  edge1.node_ids[0] = 1;
  edge1.node_ids[1] = 2;
  previous.graph.edges.push_back(edge1);

  // Copy the previous graph, then add additional entries
  auto current = previous;
  auto node3 = locus_msgs::Node();
  node3.id = 3;
  node3.position.x = 1.5;
  node3.position.y = 2.5;
  current.graph.nodes.push_back(node3);

  auto edge2 = locus_msgs::Edge();
  edge2.node_ids[0] = 2;
  edge2.node_ids[1] = 3;
  current.graph.edges.push_back(edge2);

  auto edge3 = locus_msgs::Edge();
  edge3.node_ids[0] = 1;
  edge3.node_ids[1] = 3;
  current.graph.edges.push_back(edge3);

  auto change_threshold = 0.05;
  auto result = slam_karto::computeGraphChanges(current, previous, change_threshold);

  ASSERT_EQ(result.node_changes.size(), 1u);
  EXPECT_EQ(result.node_changes[0].id, 3u);
  EXPECT_FLOAT_EQ(result.node_changes[0].position.x, 1.5f);
  EXPECT_FLOAT_EQ(result.node_changes[0].position.y, 2.5f);

  ASSERT_EQ(result.edge_changes.size(), 2u);
  EXPECT_EQ(result.edge_changes[0].node_ids[0], 2u);
  EXPECT_EQ(result.edge_changes[0].node_ids[1], 3u);
  EXPECT_EQ(result.edge_changes[1].node_ids[0], 1u);
  EXPECT_EQ(result.edge_changes[1].node_ids[1], 3u);
}

TEST(ComputeGraphChanges, MissingEntries)
{
  // Include some new entries in the current graph, but the current graph also has some missing entries relative to the
  // previous graph. The output will only include the new entries as the current message structure does not support
  // deleted entries.

  auto previous = locus_msgs::GraphStamped();
  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0;
  node1.position.y = 2.0;
  previous.graph.nodes.push_back(node1);

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1;
  node2.position.y = 2.2;
  previous.graph.nodes.push_back(node2);

  auto node3 = locus_msgs::Node();
  node3.id = 3;
  node3.position.x = 1.5;
  node3.position.y = 2.5;
  previous.graph.nodes.push_back(node3);

  auto edge1 = locus_msgs::Edge();
  edge1.node_ids[0] = 1;
  edge1.node_ids[1] = 2;
  previous.graph.edges.push_back(edge1);

  auto edge2 = locus_msgs::Edge();
  edge2.node_ids[0] = 1;
  edge2.node_ids[1] = 3;
  previous.graph.edges.push_back(edge2);

  auto edge3 = locus_msgs::Edge();
  edge3.node_ids[0] = 2;
  edge3.node_ids[1] = 3;
  previous.graph.edges.push_back(edge3);

  // Include only some of the entries from the previous graph, then add additional entries
  auto current = locus_msgs::GraphStamped();
  current.graph.nodes.push_back(node1);
  current.graph.nodes.push_back(node3);

  auto node4 = locus_msgs::Node();
  node4.id = 4;
  node4.position.x = 1.6;
  node4.position.y = 2.6;
  current.graph.nodes.push_back(node4);

  auto edge4 = locus_msgs::Edge();
  edge4.node_ids[0] = 1;
  edge4.node_ids[1] = 4;
  current.graph.edges.push_back(edge4);

  // Edges must be sorted
  current.graph.edges.push_back(edge3);

  auto edge5 = locus_msgs::Edge();
  edge5.node_ids[0] = 3;
  edge5.node_ids[1] = 4;
  current.graph.edges.push_back(edge5);

  auto change_threshold = 0.05;
  auto result = slam_karto::computeGraphChanges(current, previous, change_threshold);

  ASSERT_EQ(result.node_changes.size(), 1u);
  EXPECT_EQ(result.node_changes[0].id, 4u);
  EXPECT_FLOAT_EQ(result.node_changes[0].position.x, 1.6f);
  EXPECT_FLOAT_EQ(result.node_changes[0].position.y, 2.6f);

  ASSERT_EQ(result.edge_changes.size(), 2u);
  EXPECT_EQ(result.edge_changes[0].node_ids[0], 1u);
  EXPECT_EQ(result.edge_changes[0].node_ids[1], 4u);
  EXPECT_EQ(result.edge_changes[1].node_ids[0], 3u);
  EXPECT_EQ(result.edge_changes[1].node_ids[1], 4u);
}

TEST(ComputeGraphChanges, ChangedNodes)
{
  // Move the position of some nodes between the previous and current graph. If a node moves enough, it should be
  // reported in the output.
  auto previous = locus_msgs::GraphStamped();
  auto node1 = locus_msgs::Node();
  node1.id = 1;
  node1.position.x = 1.0;
  node1.position.y = 2.0;
  previous.graph.nodes.push_back(node1);

  auto node2 = locus_msgs::Node();
  node2.id = 2;
  node2.position.x = 1.1;
  node2.position.y = 2.2;
  previous.graph.nodes.push_back(node2);

  auto node3 = locus_msgs::Node();
  node3.id = 3;
  node3.position.x = 1.5;
  node3.position.y = 2.5;
  previous.graph.nodes.push_back(node3);

  auto current = locus_msgs::GraphStamped();
  // Node 1 moved enough for the default threshold
  node1.position.y = 2.06f;
  current.graph.nodes.push_back(node1);

  // Node 2 did not move enough
  node2.position.x = 1.14f;
  current.graph.nodes.push_back(node2);

  // Node 3 moved enough when considering both X and Y
  node3.position.x = 1.44f;
  node3.position.y = 2.54f;
  current.graph.nodes.push_back(node3);

  auto change_threshold = 0.05;
  auto result = slam_karto::computeGraphChanges(current, previous, change_threshold);

  ASSERT_EQ(result.node_changes.size(), 2u);
  EXPECT_EQ(result.node_changes[0].id, 1u);
  EXPECT_FLOAT_EQ(result.node_changes[0].position.x, 1.00f);
  EXPECT_FLOAT_EQ(result.node_changes[0].position.y, 2.06f);
  EXPECT_EQ(result.node_changes[1].id, 3u);
  EXPECT_FLOAT_EQ(result.node_changes[1].position.x, 1.44f);
  EXPECT_FLOAT_EQ(result.node_changes[1].position.y, 2.54f);

  ASSERT_EQ(result.edge_changes.size(), 0u);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package slam_karto
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.0 (2022-02-23)
-------------------
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* [RST-4712] Added checks for certain required relationships between parameters (#1)
* [RST-4384] Only publish new transforms (#2)
* [RST-4847] Add support for clockwise spinning lidars to slam_karto (#3)
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Contributors: Stephen Williams, locus-services

0.18.0 (2023-02-22)
-------------------
* 0.17.0
* Update changelogs
* Tailor: Updating Jenkinsfile
* [RST-5986] Filter out bad scans from slam_karto (#5)
* Tailor: Updating Jenkinsfile
* [RST-4712] Added checks for certain required relationships between parameters (#1)
* [RST-4384] Only publish new transforms (#2)
* [RST-4847] Add support for clockwise spinning lidars to slam_karto (#3)
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Contributors: Gary Servin, Stephen Williams, locus-services

0.19.0 (2023-09-25)
-------------------
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* [RST-7622] Refactor slam_karto to use the common GraphPublisher object (#7)
* Add missing dependency (#8)
* [RST-7375] Publish the SLAM graph as a structured message for Magellan (#5)
  * Initial version of publishing the graph structure
  * Add unit tests for applying updates to a graph
  * Moved the rviz visualization publish inside the updateMap() function as well to avoid relocking the mutex
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Rst 7062 remove map local frame (#3)
  * Removed local map frame
  * Bugfix: correct graph vertex position as all the constraints are sensor to sensor wise
  * Refactored to allow for zero offset origin correction
  * Update src/slam_karto.cpp
  Co-authored-by: Stephen Williams <stephen.vincent.williams@gmail.com>
  ---------
  Co-authored-by: Stephen Williams <stephen.vincent.williams@gmail.com>
* 0.18.0
* Update changelogs
* 0.17.0
* Update changelogs
* [RST-6486] Compile using c++17 (#1)
* [RST-5986] Filter out bad scans from slam_karto (#5)
* Tailor: Updating Jenkinsfile
* [RST-4712] Added checks for certain required relationships between parameters (#1)
* [RST-4384] Only publish new transforms (#2)
* [RST-4847] Add support for clockwise spinning lidars to slam_karto (#3)
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Contributors: Carlos Mendes, Gary Servin, Stephen Williams, locus-services

Forthcoming
-----------
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* 0.19.0
* Update changelogs
* Tailor: Updating Jenkinsfile
* [RST-7622] Refactor slam_karto to use the common GraphPublisher object (#7)
* Add missing dependency (#8)
* [RST-7375] Publish the SLAM graph as a structured message for Magellan (#5)
  * Initial version of publishing the graph structure
  * Add unit tests for applying updates to a graph
  * Moved the rviz visualization publish inside the updateMap() function as well to avoid relocking the mutex
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Rst 7062 remove map local frame (#3)
  * Removed local map frame
  * Bugfix: correct graph vertex position as all the constraints are sensor to sensor wise
  * Refactored to allow for zero offset origin correction
  * Update src/slam_karto.cpp
  Co-authored-by: Stephen Williams <stephen.vincent.williams@gmail.com>
  ---------
  Co-authored-by: Stephen Williams <stephen.vincent.williams@gmail.com>
* 0.18.0
* Update changelogs
* 0.17.0
* Update changelogs
* [RST-6486] Compile using c++17 (#1)
* [RST-5986] Filter out bad scans from slam_karto (#5)
* Tailor: Updating Jenkinsfile
* [RST-4712] Added checks for certain required relationships between parameters (#1)
* [RST-4384] Only publish new transforms (#2)
* [RST-4847] Add support for clockwise spinning lidars to slam_karto (#3)
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Contributors: Carlos Mendes, Gary Servin, Stephen Williams, locus-services

0.16.0 (2020-10-02)
-------------------
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Contributors: locus-services

0.15.0 (2019-07-12)
-------------------
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Contributors: locus-services

0.14.0 (2019-03-18)
-------------------
* [RST-372] Modified slam_karto to provide a map transform service, not listen to a rotation topic. (`#22 <https://github.com/locusrobotics/slam_karto/issues/22>`_)
  * Only publish maps when something has changed.
  * Make sure all message header stamps are synced
  * Modified slam_karto to publish the path in 'local' and 'map' frames
* Moved map alignment tool to locus_mapping_tools (`#21 <https://github.com/locusrobotics/slam_karto/issues/21>`_)
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Tailor: Updating Jenkinsfile
* Contributors: Stephen Williams, locus-services

0.13.0 (2019-01-16)
-------------------
* Tailor: Creating Jenkinsfile
* Delete the scans you aren't using (`#20 <https://github.com/locusrobotics/slam_karto/issues/20>`_)
* Contributors: Stephen Williams, locus-services

0.12.0 (2018-08-02)
-------------------
* Create the robot path message from poses in the correct frame (`#19 <https://github.com/locusrobotics/slam_karto/issues/19>`_)
* Added tf initialization back in
* Disable Werror
* Removed CXX standard 98 line
* Fixed issues until it compiled with c++14 flag
* Contributors: Paul Bovbel, Stephen Williams

0.11.1 (2018-07-10)
-------------------
* [RST-1095] fix trajectory export (`#18 <https://github.com/locusrobotics/slam_karto/issues/18>`_)
* Build in C++98
* Contributors: Paul Bovbel, Stephen Williams

0.11.0 (2018-07-05)
-------------------
* Fixed threading issue by making a deep copy of all scans (`#16 <https://github.com/locusrobotics/slam_karto/issues/16>`_)
* Contributors: Stephen Williams

0.10.0 (2018-04-16)
-------------------
* Publish the optimized trajectory as a nav_msgs::Path (`#15 <https://github.com/locusrobotics/slam_karto/issues/15>`_)
* Contributors: Stephen Williams

0.9.0 (2017-10-04)
------------------
* Added missing conversion from degree to radians
* Contributors: Stephen Williams

0.8.1 (2017-09-05)
------------------
* Reverted the automatic map origin translation. The constantly moving origin does not play well with move_base.
* Contributors: Stephen Williams

0.8.0 (2017-08-28)
------------------
* switching to standard locus release process, bumping minor version.
* Contributors: Stephen Williams

0.7.3 (2016-02-04)
------------------
* 0.7.3-5 (2017-08-28)
* Modified slam_karto to subscribe to a map rotation angle topic instead of using the tf transform.
* Modified the map alignment tool to publish the rotation angle as a topic instead of directly publishing the transform.
* 0.7.3-4 (2017-07-19)
* Synchronizing indigo and kinetic version numbers
* 0.7.3-3 (2017-07-19)
* Modified the transform check to wait a bit for the transform to arrive. It's in a separate thread, so there is little downside to waiting.
* Modified the visualization topic names
* Moved the scan queue visualization publishing into its own timer callback
* Removed the deleting of the lasers pointers. These are maintained by the dataset object.
* Modified slam_karto to always publish the map in the map_frame. Internally karto uses a local map frame for all optimizations.
* Created a map alignment node using interactive markers
* Refactored map->odom frame computation to not use tf lookups. If the optimization thread gets behind, the tf lookups can fail.
* Added a visualization of the queue size to rviz
* Modified tf and map loop times to be wall times for when the bagfile playback stops.
* Refactored slam_karto to run the mapper updates in a separate thread. A scan queue has been implemented to avoid dropping scans during long mapper updates.
* Check the service call response before updating the is_paused variable
* Modified the loop closure listener to call user-supplied functions instead of directly implementing the pause logic.
* Added a loop closure listener that publishes pause/unpause messages if the loop closure time gets too long.
* Use the map_frame\variable instead of hard-coding 'map' in the visualization message
* Reuse the map generation thread for visualization publishing as well.
* Streamlined the visualization marker creation system
* Added a separate thread for publishing the visualization markers
* Precomputed map index, removed unneeded map index macro
* Moved the map generation code to run in a separate thread.
* Clean up lasers map in destructor.
* Fixed locks so they stay in scope until the end of the method.
* modify for stage simulation
* 0.7.3-2
* Being a bit more robust about transform handling
* Changing behavior when a transform from odom->base_link fails
* 0.7.3-1
* Parameterizing sparse pose adjustment method
* Update karto_slam.launch
* Parameterizing sparse pose adjustment method
* Update karto_slam.launch
* Contributors: Harsh Pandya, Michael Ferguson, Tom Moore
* 0.7.3 (2016-02-04)
* fixed the upside-down detection
* update maintainer email
* Contributors: Michael Ferguson, mgerdzhev

0.7.2 (2015-07-18)
------------------
* Added in parameter server settings for Mapper within slam_karto
* Contributors: Luc Bettaieb, Michael Ferguson

0.7.1 (2014-06-17)
------------------
* build updates for sba, fix install
* Contributors: Michael Ferguson

0.7.0 (2014-06-15)
------------------
* First release in a very, very long time.
* Catkinized, updated to work with catkinized open_karto and sba
* Contributors: Jon Binney, Michael Ferguson

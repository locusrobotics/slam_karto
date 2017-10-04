^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package slam_karto
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

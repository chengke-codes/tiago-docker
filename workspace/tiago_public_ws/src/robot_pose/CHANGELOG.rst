^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_pose
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2021-12-15)
------------------
* Merge branch 'fix-lookup-transform' into 'erbium-devel'
  fix lookup transform
  See merge request navigation/robot_pose!16
* updated comment
* fixed lookup transform
* Contributors: josegarcia

1.0.8 (2021-12-14)
------------------
* Merge branch 'feature-transform-timeout-as-parameter' into 'erbium-devel'
  Added transform timeout as parameter
  See merge request navigation/robot_pose!15
* added transform timeout as parameter
* Contributors: Jordan Palacios, josegarcia

1.0.7 (2020-04-21)
------------------
* Merge branch 'fix-nodehandle' into 'erbium-devel'
  use global nodehandle
  See merge request navigation/robot_pose!13
* use global nodehandle
* Contributors: Procópio Stein, procopiostein

1.0.6 (2020-04-16)
------------------
* added namespace and play bag in a loop
  also changed name and location of rosbag
* Fix test and add rosbag for input
  Rename launch file
* Add tests for fake_odom
* removed slash from topic name to allow namespacing
* fixed launch filename and args
* Parametrized fake odom
* use standard odom topic name
* fixed publisher
* renamed node
* fixed structure
* Add fake pose publisher for gazebo
* Contributors: Procópio Stein, Sara Cooper, procopiostein

1.0.5 (2019-08-23)
------------------
* Fix build errors
* Contributors: Victor Lopez

1.0.4 (2019-08-14)
------------------
* Merge branch 'prefix-fix' into 'erbium-devel'
  Don't use prefix on output topic, it should be namespaced
  See merge request navigation/robot_pose!11
* Don't use prefix on output topic, it should be namespaced
* Contributors: Victor Lopez

1.0.3 (2019-08-14)
------------------
* Merge branch 'multi_robot' into 'erbium-devel'
  added prefix to topics and tf frames for multi robot usage
  See merge request navigation/robot_pose!10
* parameterize the frames and robot prefix
* added prefix to topics and tf frames for multi robot usage
* Merge branch 'exetend_statistics' into 'erbium-devel'
  Use Odom info to compute travelled distance
  See merge request navigation/robot_pose!9
* Fix test to use odom
* Use Odom info to compute travelled distance
* Contributors: Procópio Stein, Sai Kishor Kothakota, davidfernandez

1.0.2 (2018-10-24)
------------------
* Merge branch 'improve-constness' into 'erbium-devel'
  pal_statistics changed API
  See merge request navigation/robot_pose!7
* pal_statistics changed API
* Contributors: Victor Lopez

1.0.1 (2018-10-23)
------------------
* Merge branch 'tf2-migration-erbium' into 'erbium-devel'
  Tf2 migration erbium
  See merge request navigation/robot_pose!5
* catch as &
* added license
* added robot_pose test
  Conflicts:
  CMakeLists.txt
* do not publish if there is a tf exception
* added missing dependencies
* reduced freq to 50
* split tf lookup into two, to avoid jumps due to amcl initialpose
  this happened when slippaged detector published several initialpose
  messages. we found out that splitting the lookup from
  map -> base_footprint to map -> odom -> base_footprint
  solved the issue
* migration from tf to tf2
* Contributors: Procópio Stein, Victor Lopez

1.0.0 (2018-07-18)
------------------
* Merge branch 'add-distance-travelled' into 'erbium-devel'
  Add distance travelled publisher
  See merge request navigation/robot_pose!4
* Replace amcl_pose with robot_pose
* Add distance travelled publisher
* cosmetic, matching pal code style
* upgrade package to format 2, cleaned cmakelists
* Contributors: Procópio Stein, Victor Lopez

0.0.2 (2016-03-04)
------------------
* Merge branch 'creation' into 'master'
  changed publisher queue size and fixed authors name
  fixed queue size and authors name
  See merge request !1
* changed publisher queue size and fixed authors name
* fixed tranform direction
* robot pose node implemented
* added readme
* Contributors: Procopio Stein, Victor Lopez

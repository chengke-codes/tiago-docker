^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_models_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2018-06-12)
------------------
* Merge branch 'as_fixes_tests' into 'erbium-devel'
  Add tests, follow conventions regarding launch files.
  See merge request robots/simple_models_robot!3
* Add tests, follow conventions regarding launch files.
* Contributors: Hilario Tome, alexandersherikov

0.0.31 (2022-01-26)
-------------------
* Merge branch 'realsense-fix-test' into 'erbium-devel'
  Realsense fix test
  See merge request robots/simple_models_robot!36
* Fixed navigation test with camera_model parameter
* Contributors: Jordan Palacios, antoniobrandi

0.0.30 (2021-12-10)
-------------------
* Merge branch 'fix-simple-pmb2-odom-topic' into 'erbium-devel'
  Fix topic for pmb2 odom
  See merge request robots/simple_models_robot!34
* Fix topic for pmb2 odom
* Contributors: Victor Lopez, victor

0.0.29 (2021-09-17)
-------------------
* Another xacro instead of xacro.py
* Contributors: Jordan Palacios

0.0.28 (2021-09-01)
-------------------
* Merge branch 'gallium_fixes' into 'erbium-devel'
  Gallium fixes
  See merge request robots/simple_models_robot!32
* Using xacro instead of deprecated xacro.py
* Contributors: Jordan Palacios

0.0.27 (2021-09-01)
-------------------

0.0.26 (2021-03-10)
-------------------
* Merge branch 'kangaroo_transmission' into 'erbium-devel'
  added a box link and joint with transmission for pal physics simulator testing
  See merge request robots/simple_models_robot!31
* remove simple_stockbot test
* added a box link and joint with transmission for pal physics simulator testing
* Removed stockbot_description because it breaks public simulation
* Contributors: Sai Kishor Kothakota, Victor Lopez, victor

0.0.25 (2021-02-08)
-------------------
* Merge branch 'fleet-fixes' into 'erbium-devel'
  Ignore model_name in simple_models robot
  See merge request robots/simple_models_robot!29
* Add missing test dependency
* Don't use base sensors if not pmb2 or stockbot
* Ignore model_name in simple_models robot
* Contributors: Victor Lopez, victor

0.0.24 (2021-01-15)
-------------------
* Merge branch 'sick_tim571' into 'erbium-devel'
  stockbot laser updated
  See merge request robots/simple_models_robot!28
* stockbot laser updated
* Contributors: noeperez, victor

0.0.23 (2020-07-09)
-------------------
* Merge branch 'safety' into 'erbium-devel'
  Add velocity and effort limits for pal_local_joint_control
  See merge request robots/simple_models_robot!27
* Add velocity and effort limits for pal_local_joint_control
* Contributors: Adria Roig, victor

0.0.22 (2020-04-17)
-------------------
* Merge branch 'fix-launch-arg' into 'erbium-devel'
  fix missing default arg for upload.launch
  See merge request robots/simple_models_robot!26
* fix missing default arg for upload.launch
* Contributors: Procópio Stein, victor

0.0.21 (2020-04-07)
-------------------
* Merge branch 'fix-tests' into 'erbium-devel'
  Fix tests
  See merge request robots/simple_models_robot!24
* Revert "simplified args and fixed test"
  This reverts commit 6c709cacef3b7b4152b9117500792d6eaa0f76c0.
* Contributors: Procópio Stein, procopiostein

0.0.20 (2020-04-06)
-------------------

0.0.19 (2020-04-06)
-------------------
* Merge branch 'simple_stockbot_model' into 'erbium-devel'
  add simple stockbot model
  See merge request robots/simple_models_robot!21
* added simple stockbot test
* simplified args and fixed test
* fixed frames order
* updated simple stockbot urdf xacro
* separate sensors files for stockbot
* remove stockbot wheels and update mass and origin
* adapt robot.urdf.xacro according to the robot
* add robot_name and change wheel joint type to fixed in stockbot simple model
* add simple stockbot model
* Contributors: Adria Roig, Procópio Stein, YueErro

0.0.18 (2020-01-14)
-------------------

0.0.17 (2019-10-30)
-------------------
* Merge branch 'ivo_plugins' into 'erbium-devel'
  Add four wheel plugin
  See merge request robots/simple_models_robot!19
* Add four wheel plugin
* Contributors: Adria Roig

0.0.16 (2019-10-21)
-------------------
* Merge branch 'fix-frame-ns' into 'erbium-devel'
  fixed using model name as tf prefixs
  See merge request robots/simple_models_robot!18
* gazebo laser outputs scan_raw instead of scan
* simple model vel input now is mobile_base_controller/cmd_vel now
* fixed using model name as tf prefixs
* Contributors: Procópio Stein

0.0.15 (2019-09-19)
-------------------
* Merge branch 'ferrum-fixes' into 'erbium-devel'
  Ferrum compatibility. 'false' is read as False in melodic
  See merge request robots/simple_models_robot!16
* Ferrum compatibility. 'false' is read as False in melodic
* Contributors: Victor Lopez

0.0.14 (2019-09-06)
-------------------
* Merge branch 'friction' into 'erbium-devel'
  Friction
  See merge request robots/simple_models_robot!15
* Add gravity and remove friction
* Contributors: Adria Roig, Victor Lopez

0.0.13 (2019-07-18)
-------------------
* Merge branch 'multi_pmb2' into 'erbium-devel'
  Change for muliple pmb2 navigation
  See merge request robots/simple_models_robot!11
* Change for muliple pmb2 navigation
* Contributors: Adria Roig, Victor Lopez

0.0.12 (2019-07-04)
-------------------
* Merge branch 'melodic-devel' into 'erbium-devel'
  Melodic devel
  See merge request robots/simple_models_robot!10
* Upload with no laser by default
* Contributors: Adria Roig, Victor Lopez

0.0.11 (2019-07-02)
-------------------

0.0.10 (2019-07-02)
-------------------
* Merge branch 'simple_sim' into 'erbium-devel'
  Create pmb2 simple sim model
  See merge request robots/simple_models_robot!8
* Remove Media / worlds / models + Fix tests
* Add box. Fix tests
* Create pmb2 simple sim model
* Contributors: Adria Roig, Victor Lopez

0.0.9 (2019-03-18)
------------------

0.0.8 (2019-01-19)
------------------
* Add missing dependency
* Contributors: Victor Lopez

0.0.7 (2018-11-29)
------------------
* Merge branch 'as_safety' into 'erbium-devel'
  Add safety parameters. Rename config files to match joint names.
  See merge request robots/simple_models_robot!6
* Add safety parameters. Rename config files to match joint names.
* Contributors: alexandersherikov

0.0.6 (2018-10-17)
------------------

0.0.5 (2018-07-16)
------------------
* 0.0.4
* Updated changelog
* Merge branch 'as_fixes_tests' into 'erbium-devel'
  Add tests, follow conventions regarding launch files.
  See merge request robots/simple_models_robot!3
* Add tests, follow conventions regarding launch files.
* Contributors: Hilario Tome, alexandersherikov

0.0.3 (2018-06-07)
------------------

0.0.2 (2018-06-07)
------------------
* Merge branch 'install_rules' into 'master'
  added install rules
  See merge request robots/simple_models_robot!2
* added install rules
* Contributors: Hilario Tome

0.0.1 (2018-06-07)
------------------
* added sphere
* Merge branch 'as_more_models' into 'master'
  As more models
  See merge request robots/simple_models_robot!1
* Added two more models
* fixes
* moved intro _description and controller_configuration packages
* Contributors: Hilario Tome, alexandersherikov

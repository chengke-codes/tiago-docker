# Custom End Effector
This repository contains a set of prepared packages and files which can be used to integrate a custom end-effector to a TIAGo/TIAGo++ robot. Once the apropriate modifications are made, deploy all three packages to the robot and select `custom` for the end effector type on the Web Commander.

## Packages
This repository is comprised of three different packages (3 for each TIAGo model), which conatin a wrapper for the URDF description of the end-effector, configuration files and MoveIt configuration for the new configuration.

**TIAGo++**
To make your lives easier, we added in each of the following packages some comments inside files in order to let you find faster what to modify. Look for `TODO:` keyword to localize the tips.

### custom_ee_description / custom_dual_ee_description
This package contains a simple wrapper to include the URDF description of the custom end-effector, so the system knows its new configuration. This package must contain the following file/s:
**TIAGo**
* `urdf/end-effector.urdf.xacro`
  * This file must contain a xacro macro named `end_effector`, with a paramater named `parent`, which contains the name of the previous link in the kinematic chain of the arm; and an `origin` tag as its content, with the root position of the custom end-effector.

**TIAGo++**
* `urdf/end-effector-<side>.urdf.xacro`
  * This files must contain a xacro macro named `end_effector_<side>`, with a paramater named `parent`, which contains the name of the previous link in the kinematic chain of the arm; and an `origin` tag as its content, with the root position of the custom end-effector.

### moveit_custom_config / moveit_custom_dual_config
This package contains configuration files need for MoveIt to interact properly with the new end-effector.


**TIAGo**

* `controllers/controllers_custom.yaml`
  * This file contains a list of controllers known to MoveIt. The default `arm_controller`, `torso_controller` and `head_controller` should be left on the file, but additional controllers required by the end-effector can be added here.

*  `srdf/tiago_custom.srdf`
   * This file contains the `srdf` definition for the complete robot, defining move groups and blacklisted collisions. This file can be generated using the `moveit_setup_assistant` package, by loading the file `robots/tiago.urdf.xacro` in the `tiago_description` package and adding the `end_effector:=custom` argument.

**TIAGo++**
* `controllers/controllers_custom.yaml`
  * This file contains a list of controllers known to MoveIt. The default `left_arm_controller`, `right_arm_controller`, `torso_controller` and `head_controller` should be left on the file, but additional controllers required by the end-effector can be added here.

*  `srdf/tiago_dual.srdf.em`
   * This file contains the `srdf` definition for the complete robot, defining move groups and blacklisted collisions. To create your desired combinations of end effectors you need to modify the file according to your custom end effector as now is using a simple cilinder as an example.

   After modifying it, you can generate all possible combinations of end effectors using the script `custom_end_effector/moveit_custom_dual_config/scripts/regen_em_file.py` like this `python regen_em_file.py <path-to-tiago_dual.srdf.em>`
   
   Alternatively, this file can be generated using the `moveit_setup_assistant` package, by loading the file `robots/tiago_dual.urdf.xacro` in the `tiago_dual_description` package and adding the `end_effector_<side>:=custom` argument for each side. Then you only need to rename to follow properly these format: `tiago_dual_<end-effector-left>_<end-effector-right>.srdf`.

### custom_ee_configuration / custom_dual_ee_configuration
This package contains other configuration and launch files used by the TIAGo internal architecture.
* `config/approach_planner/approach_planner.yaml`
  * Contains paramters for `play_motion`, both planning groups to be used when planning the approach to the starting position via MoveIt, and the joints to be excluded (those not controlled via MoveIt).

* `config/motions/tiago_motions.yaml / config/motions/tiago_dual_custom_motions.yaml`
  **ATTENTION:**By default these motions only check for collisions the first specified waypoint in the motion! This means that if your end effector differs a lot from ours, you should ensure first that it is not going to collide.

**TIAGo**
   * Contains predefined motions for the TIAGo robot. This motions may be modified as needed, but it is strongly recommended to keep at least the original `home` motion included in the file.

**TIAGo++**
   * You should add motions related to your end effector. All combinations for other end effectors and parts of the body are defined inside `$(rospack find tiago_dual_bringup)/config/motions`.

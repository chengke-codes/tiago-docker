#! /bin/sh
#
# Check if the $HOME/.pal folder is available and creates it if needed before
# launching navigation.
#
# Usage: $0 <robot> [<state>] [<localization method>] [<mapping method>] [<map>]

# Check parameters:
if [ $# -lt 1 ]; then
  echo "Usage: $0 <robot> [<state>] [<localization method>] [<mapping method>] [<map>] [<octomap>]"
  echo "Check if the $HOME/.pal folder is availabe and creates it if needed"
  echo "before launching navigation."
  exit 1
else
  ROBOT=$1
fi

if [ $# -lt 2 ]; then
  STATE=localization
else
  STATE=$2
fi

if [ $# -lt 3 ]; then
  LOCALIZATION=amcl
else
  LOCALIZATION=$3
fi

if [ $# -lt 4 ]; then
  MAPPING=gmapping
else
  MAPPING=$4
fi

if [ $# -lt 5 ]; then
  MAP=$HOME/.pal/${ROBOT}_maps/config
else
  MAP=$5
fi

#if [ -d "$MAP" -a "$STATE" = "mapping" ]; then
#  rm -rf $MAP
#fi

if [ $# -lt 7 ]; then
  MULTI="false"
  ROBOT_NAMESPACE=""
else 
  if [ "$7" = "true" ]; then
    if [ $# -lt 8 ]; then
      echo "If MULTI is true I need the robot_namespace"
      exit 1
    else
      MULTI="true"
      ROBOT_NAMESPACE=$8
    fi
  else 
    MULTI="false"
    ROBOT_NAMESPACE=""
  fi
fi

# Ensure target directory exists
if [ ! -d "$MAP" ]; then
  mkdir -p $MAP
  if [ $? -ne 0 ]; then
   echo "Error: Failed to create path $MAP"
    exit 3
  fi
fi

if [ ! -f "$HOME/.pal/pose.yaml" ]; then
    rosrun pal_navigation_sm cp_pose_to_home.sh
fi

# Run localization/mapping
roslaunch ${ROBOT}_2dnav_gazebo $STATE.launch localization:=$LOCALIZATION mapping:=$MAPPING map:=$MAP multiple:=$MULTI robot_namespace:=$ROBOT_NAMESPACE




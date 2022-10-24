/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef GAZEBOFOURWHEELDIFFDRIVEPLUGIN_H
#define GAZEBOFOURWHEELDIFFDRIVEPLUGIN_H


#include <simple_models_gazebo/gazebo_diff_drive.h>

namespace gazebo
{
class GazeboFourWheeldDiffDrivePlugin : public ModelPlugin
{
public:
  GazeboFourWheeldDiffDrivePlugin();
  ~GazeboFourWheeldDiffDrivePlugin();
  virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
  virtual void Reset() override;

protected:
  virtual void UpdateChild();
  virtual void FiniChild();

  GazeboDiffDrive diff_drive_;

  std::string left_front_joint_wheel_;
  std::string right_front_joint_wheel_;
  std::string left_rear_joint_wheel_;
  std::string right_rear_joint_wheel_;
};
}

#endif // GAZEBOFOURWHEELDIFFDRIVEPLUGIN_H

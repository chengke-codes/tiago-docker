/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef GAZEBO_DIFF_DRIVE_PLUGIN_H
#define GAZEBO_DIFF_DRIVE_PLUGIN_H

#include <simple_models_gazebo/gazebo_diff_drive.h>

namespace gazebo
{
class GazeboDiffDrivePlugin : public ModelPlugin
{
public:

  GazeboDiffDrivePlugin();
  ~GazeboDiffDrivePlugin();
  virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
  virtual void Reset() override;

protected:
  virtual void UpdateChild();
  virtual void FiniChild();

  GazeboDiffDrive diff_drive_;

  bool gravity_;
  double Kp_;
  double Kw_;

//  tf::Quaternion prev_qt_;
//  tf::Vector3 prev_vt_;
};
}

#endif // GAZEBO_DIFF_DRIVE_PLUGIN_H

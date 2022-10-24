/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <simple_models_gazebo/gazebo_four_wheel_diff_drive_plugin.h>

namespace gazebo
{
GazeboFourWheeldDiffDrivePlugin::GazeboFourWheeldDiffDrivePlugin()
{
}

// Destructor
GazeboFourWheeldDiffDrivePlugin::~GazeboFourWheeldDiffDrivePlugin()
{
}

// Load the controller
void GazeboFourWheeldDiffDrivePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  diff_drive_.LoadGeneric(_parent, _sdf);
  diff_drive_.gazebo_ros_->getParameter<std::string>(left_front_joint_wheel_, "leftFrontJoint",
                                                     "wheel_front_left_joint");
  diff_drive_.gazebo_ros_->getParameter<std::string>(
      right_front_joint_wheel_, "rightFrontJoint", "wheel_front_right_joint");
  diff_drive_.gazebo_ros_->getParameter<std::string>(left_rear_joint_wheel_, "leftRearJoint",
                                                     "wheel_back_left_joint");
  diff_drive_.gazebo_ros_->getParameter<std::string>(right_rear_joint_wheel_, "rightRearJoint",
                                                     "wheel_back_right_joint");

  // listen to the update event (broadcast every simulation iteration)
  diff_drive_.update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboFourWheeldDiffDrivePlugin::UpdateChild, this));
}

void GazeboFourWheeldDiffDrivePlugin::FiniChild()
{
  diff_drive_.FiniChildGeneric();
}

void GazeboFourWheeldDiffDrivePlugin::Reset()
{
  diff_drive_.ResetGeneric();
}

void GazeboFourWheeldDiffDrivePlugin::UpdateChild()
{
  boost::mutex::scoped_lock scoped_lock(diff_drive_.lock);

  diff_drive_.UpdateChildGeneric();

  //  if (diff_drive_.seconds_since_last_update_ > diff_drive_.update_period_)
  {
    double w = 2 * diff_drive_.x_ / diff_drive_.wheel_diameter_;
    diff_drive_.model_->SetAngularVel(ignition::math::Vector3d(0, 0, diff_drive_.rot_));
    diff_drive_.model_->GetJoint(left_front_joint_wheel_)->SetVelocity(0, w);
    diff_drive_.model_->GetJoint(right_front_joint_wheel_)->SetVelocity(0, w);
    diff_drive_.model_->GetJoint(left_rear_joint_wheel_)->SetVelocity(0, w);
    diff_drive_.model_->GetJoint(right_rear_joint_wheel_)->SetVelocity(0, w);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFourWheeldDiffDrivePlugin)
}

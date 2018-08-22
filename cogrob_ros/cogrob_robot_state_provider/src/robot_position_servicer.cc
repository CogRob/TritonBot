// Copyright (c) 2018, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
// * Neither the name of the University of California nor the
//   names of its contributors may be used to endorse or promote products
//   derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robot_position_servicer.h"

#include "absl/memory/memory.h"
#include "absl/synchronization/mutex.h"

namespace cogrob_robot_state_provider {

RobotPositionServicer::RobotPositionServicer(ros::NodeHandle* ros_node) {
  auto get_robot_position_service = [this] (
    cogrob_robot_state_msgs::GetRobotPosition::Request& request,
    cogrob_robot_state_msgs::GetRobotPosition::Response& response) -> bool {
    return this->GetRobotPositionServiceCallbak(request, response);
  };
  get_robot_position_srv_ = absl::make_unique<ros::ServiceServer>(
      ros_node->advertiseService<
      cogrob_robot_state_msgs::GetRobotPosition::Request,
      cogrob_robot_state_msgs::GetRobotPosition::Response>(
      "/cogrob/get_robot_position", get_robot_position_service));
}

bool RobotPositionServicer::GetRobotPositionServiceCallbak(
    cogrob_robot_state_msgs::GetRobotPosition::Request& request,
    cogrob_robot_state_msgs::GetRobotPosition::Response& response) {
  absl::MutexLock lock(&mutex_);
  if (!tf_listener_) {
    tf_listener_ = absl::make_unique<tf::TransformListener>();
    try {
      tf_listener_->waitForTransform(
          "map", "base_link", ros::Time(0), ros::Duration(1.0));
    } catch (tf::TransformException& tf_exception) {
      tf_listener_.reset(nullptr);
    }
  }

  if (!tf_listener_) {
    response.is_error = true;
    response.error_message = "TF initialize error.";
    return true;
  }

  tf::StampedTransform current_tf;
  try {
    ros::Time now = ros::Time::now();
    tf_listener_->waitForTransform("map", "base_link", now, ros::Duration(1.0));
    tf_listener_->lookupTransform("map", "base_link", now, current_tf);
  } catch (tf::TransformException& tf_exception) {
    response.is_error = true;
    response.error_message = "TF lookup transform error.";
    tf_listener_.reset(nullptr);
    return true;
  }

  response.world_x = current_tf.getOrigin().x();
  response.world_y = current_tf.getOrigin().y();
  response.orientation = tf::getYaw(current_tf.getRotation());

  return true;
}

}  // namespace cogrob_robot_state_provider

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

#include <grpc++/grpc++.h>
#include <string>

#include "battery_state_servicer.h"
#include "robot_position_servicer.h"
#include "robot_status_impl.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cogrob_robot_state_provider");
  ros::NodeHandle ros_node;

  cogrob_robot_state_provider::BatteryStateServicer battery_state(&ros_node);
  ROS_INFO("GetBatteryStatus service registered.");

  cogrob_robot_state_provider::RobotPositionServicer robot_position(&ros_node);
  ROS_INFO("GetRobotPosition service registered.");

  const std::string GRPC_SERVER_ADDRESS = "0.0.0.0:7012";

  cogrob_robot_state_provider::RobotStatusImpl grpc_service(&battery_state);
  grpc::ServerBuilder builder;
  builder.AddListeningPort(
      GRPC_SERVER_ADDRESS, grpc::InsecureServerCredentials());
  builder.RegisterService(&grpc_service);
  std::unique_ptr<grpc::Server> grpc_server(builder.BuildAndStart());

  ros::spin();
  return 0;
}

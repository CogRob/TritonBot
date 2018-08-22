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

#include <memory>

#include "absl/memory/memory.h"
#include "cogrob/universal_logger/universal_logger.h"
#include "cogrob/universal_logger/universal_logger_flusher.h"
#include "cogrob/universal_logger/universal_logger_interface.h"
#include "cogrob_robot_state_msgs/GetRobotPosition.h"
#include "ros/ros.h"
#include "fetch_auto_dock_msgs/DockActionGoal.h"
#include "fetch_auto_dock_msgs/DockActionResult.h"
#include "fetch_auto_dock_msgs/UndockActionGoal.h"
#include "fetch_auto_dock_msgs/UndockActionResult.h"
#include "third_party/gflags.h"
#include "third_party/glog.h"

#include "auto_dock_logger.h"

using auto_dock_logging::AutoDockLogger;
using auto_dock_logging::GetRobotPositionProxy;
using cogrob::universal_logger::UniversalLogger;
using cogrob::universal_logger::UniversalLoggerFlusher;
using cogrob::universal_logger::UniversalLoggerInterface;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "auto_dock_logger_node");
  ros::NodeHandle ros_node;

  ros::ServiceClient get_robot_position_client =
      ros_node.serviceClient<cogrob_robot_state_msgs::GetRobotPosition>(
      "/cogrob/get_robot_position");
  GetRobotPositionProxy get_robot_position_proxy(&get_robot_position_client);

  UniversalLoggerFlusher logger_flusher;
  UniversalLogger universal_logger("navigation/auto_dock");
  CHECK_OK(universal_logger.RegisterWithFlusher(&logger_flusher));
  AutoDockLogger auto_dock_logger(&universal_logger, &get_robot_position_proxy);

  auto dock_goal_callback = [&auto_dock_logger] (
      const fetch_auto_dock_msgs::DockActionGoal::ConstPtr& msg) {
    auto_dock_logger.DockActionGoalCallback(msg);
  };
  ros::Subscriber dock_goal_sub =
      ros_node.subscribe<fetch_auto_dock_msgs::DockActionGoal>(
      "/dock/goal", 10, dock_goal_callback);

  auto dock_result_callback = [&auto_dock_logger] (
      const fetch_auto_dock_msgs::DockActionResult::ConstPtr& msg) {
    auto_dock_logger.DockActionResultCallback(msg);
  };
  ros::Subscriber dock_result_sub =
      ros_node.subscribe<fetch_auto_dock_msgs::DockActionResult>(
      "/dock/result", 10, dock_result_callback);

  auto undock_goal_callback = [&auto_dock_logger] (
      const fetch_auto_dock_msgs::UndockActionGoal::ConstPtr& msg) {
    auto_dock_logger.UndockActionGoalCallback(msg);
  };
  ros::Subscriber undock_goal_sub =
      ros_node.subscribe<fetch_auto_dock_msgs::UndockActionGoal>(
      "/undock/goal", 10, undock_goal_callback);

  auto undock_result_callback = [&auto_dock_logger] (
      const fetch_auto_dock_msgs::UndockActionResult::ConstPtr& msg) {
    auto_dock_logger.UndockActionResultCallback(msg);
  };
  ros::Subscriber undock_result_sub =
      ros_node.subscribe<fetch_auto_dock_msgs::UndockActionResult>(
      "/undock/result", 10, undock_result_callback);

  ros::spin();
  return 0;
}

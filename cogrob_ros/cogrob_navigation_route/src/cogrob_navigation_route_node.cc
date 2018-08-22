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

#include <unistd.h>

#include "absl/memory/memory.h"
#include "absl/synchronization/mutex.h"
#include "actionlib/server/simple_action_server.h"
#include "cogrob_navigation_msgs/MoveToNavPointAction.h"
#include "grpc++/grpc++.h"
#include "move_control.h"
#include "ros/ros.h"
#include "third_party/glog.h"
#include "third_party/gflags.h"
#include "util/status.h"

namespace {

void MoveToNavPointActionCallback(
    const std::string& target,
    cogrob_navigation_route::MoveControlInterface* move_control,
    actionlib::SimpleActionServer<
    cogrob_navigation_msgs::MoveToNavPointAction>* action_server) {
  CHECK_OK(move_control->StartMoveToNavPoint(target));

  bool preempt_requested = false;
  while (move_control->IsRunning()) {
    // TODO(shengye): Provide feedback.
    if (action_server->isPreemptRequested() || !ros::ok()) {
      LOG(ERROR) << "Preempt Requested";
      preempt_requested = true;
      CHECK_OK(move_control->ForceCancel());
    }
    usleep(100000);
  }

  const util::Status mc_result = move_control->GetResult();
  cogrob_navigation_msgs::MoveToNavPointResult result;
  if (mc_result.ok()) {
    result.is_success = true;
  } else {
    result.is_success = false;
  }
  result.error_msg = mc_result.error_message();
  if (preempt_requested) {
    action_server->setPreempted(result);
  } else {
    action_server->setSucceeded(result);
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  grpc_init();

  ros::init(argc, argv, "cogrob_navigation_route_action_server");
  ros::NodeHandle ros_node_handle;

  cogrob_navigation_route::MoveControl move_control(&ros_node_handle);

  absl::Mutex action_server_mutex;
  std::unique_ptr<actionlib::SimpleActionServer<
      cogrob_navigation_msgs::MoveToNavPointAction>> action_server;

  std::function<void (
      const cogrob_navigation_msgs::MoveToNavPointGoalConstPtr &)>
      exec_callback =
      [&action_server_mutex, &action_server, &move_control]
      (const cogrob_navigation_msgs::MoveToNavPointGoalConstPtr& goal) {
          absl::ReaderMutexLock lock(&action_server_mutex);
          if (action_server) {
            MoveToNavPointActionCallback(
                goal->target_nav_point, &move_control, action_server.get());
          } else {
            LOG(ERROR) << "Action goal received before ready, ignorring goal";
          }
      };

  {
    absl::WriterMutexLock lock(&action_server_mutex);
    action_server = absl::make_unique<actionlib::SimpleActionServer<
        cogrob_navigation_msgs::MoveToNavPointAction>>(
        ros_node_handle, "/cogrob/move_to_nav_point", exec_callback, false);
  }

  action_server->start();
  LOG(INFO) << "CogRob Navigation Route action server started.";
  ros::spin();
  return 0;
}

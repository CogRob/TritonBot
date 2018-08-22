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

#ifndef COGROB_NAVIGATION_ROUTE_MOVE_CONTROL_H_
#define COGROB_NAVIGATION_ROUTE_MOVE_CONTROL_H_

#include <memory>
#include <string>
#include <thread>

#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "cogrob/universal_logger/universal_logger_interface.h"
#include "cogrob/universal_logger/universal_logger_flusher_interface.h"
#include "cogrob/navigation/proto/logging.pb.h"
#include "cogrob/navigation/traffic_control/proto/traffic_control.grpc.pb.h"
#include "dock_control_interface.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_control_interface.h"
#include "ros/ros.h"
#include "third_party/glog.h"
#include "third_party/gflags.h"
#include "util/status.h"
#include "util/statusor.h"

namespace cogrob_navigation_route {

enum class MoveControlError {
  UNKNOWN, OK, CANCELLED, UNDOCK_FAILURE, DOCK_FAILURE, MOVEBASE_TIMEOUT,
  UNEXPECTED_ACTION_STATE, TRAFFIC_CONTROL_FAIL, GET_ROBOT_POSITION_FAIL
};

class MoveControl : public MoveControlInterface {
 public:
  MoveControl(ros::NodeHandle* ros_node);
  util::Status ForceCancel() override;
  util::Status GetResult() override;
  bool IsRunning() override;

  // TODO(shengye): Provide feedback.
  util::Status StartMoveToNavPoint(const std::string& nav_point) override;
 private:
  void MoveToNavPointThread(std::string nav_point);

  util::Status GoToNearCoordinate(double x, double y, double yaw,
      double tolerance, bool auto_cancel, bool wait_finish,
      MoveControlError* move_control_error);

  util::Status DirectGoTo(const std::string& nav_point, bool use_nav_point_yaw,
      double tolerance, bool auto_cancel, bool wait_finish,
      MoveControlError* move_control_error);

  util::Status MoveToLocation(
      const std::string& nav_point, MoveControlError* move_control_error);

  util::Status GoHomeAndDock(MoveControlError* move_control_error);

  util::StatusOr<std::string> GetMapName();

  util::Status PopulateCurrentLocation(cogrob::navigation::Location* location,
                                       MoveControlError* move_control_error);

  util::Status PopulateNavPointInfo(
      const std::string& nav_point_name,
      cogrob::navigation::NavPoint* nav_point,
      MoveControlError* move_control_error);

  std::unique_ptr<actionlib::SimpleActionClient<
      move_base_msgs::MoveBaseAction>> move_base_;
  std::unique_ptr<ros::ServiceClient> robot_position_client_;
  std::unique_ptr<DockControlInterface> dock_control_;
  std::unique_ptr<cogrob::navigation::traffic_control::TrafficControl::Stub>
      traffic_control_stub_;

  std::atomic_bool force_stop_ {false};
  std::atomic_bool is_running_ {false};

  std::unique_ptr<cogrob::universal_logger::UniversalLoggerInterface>
      direct_go_to_logger_;
  std::unique_ptr<cogrob::universal_logger::UniversalLoggerInterface>
      move_to_location_logger_;
  std::unique_ptr<cogrob::universal_logger::UniversalLoggerInterface>
      go_home_and_dock_logger_;

  // Must be declared after loggers, so it will be destructed before the
  // loggers.
  std::unique_ptr<cogrob::universal_logger::UniversalLoggerFlusherInterface>
      log_flusher_;

  std::unique_ptr<std::thread> move_thread_;
  absl::Mutex mutex_;  // Prevents public method reentry;
  absl::Mutex latest_status_mutex_;
  util::Status latest_status_ GUARDED_BY(latest_status_mutex_);
};

}  // namespace cogrob_navigation_route

#endif  // COGROB_NAVIGATION_ROUTE_MOVE_CONTROL_H_

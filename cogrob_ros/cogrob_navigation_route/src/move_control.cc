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

#include "move_control.h"

#include <cmath>

#include "absl/memory/memory.h"
#include "cogrob_robot_state_msgs/GetRobotPosition.h"
#include "cogrob/universal_logger/event_entry_helper.h"
#include "cogrob/universal_logger/universal_logger.h"
#include "cogrob/universal_logger/universal_logger_flusher.h"
#include "dock_control.h"
#include "grpc++/grpc++.h"
#include "third_party/gflags.h"
#include "util/status_macros.h"
#include "util/timestamp.h"

using cogrob::navigation::traffic_control::TrafficControl;
using cogrob::universal_logger::UniversalLogger;
using cogrob::universal_logger::UniversalLoggerFlusher;
using cogrob::universal_logger::EventEntryHelper;
using util::Status;
using util::StatusOr;

DEFINE_double(
    move_waypoint_tolerance_distance, 1.0,
    "Tolerance for moving to a waypoint.");
DEFINE_double(
    move_destination_tolerance_distance, 0.1,
    "Tolerance for moving to a destination.");
DEFINE_bool(
    move_destination_cancel_goal, false,
    "Whether to cancel go after moving to a destination.");
DEFINE_bool(
    move_destination_wait_finish, false,
    "Whether to wait move_base to finish when moving to a destination.");

namespace traffic_control = cogrob::navigation::traffic_control;

namespace cogrob_navigation_route {

namespace {

cogrob::navigation::FailReason MoveControlErrorToFailReason(
    MoveControlError mc_error) {
  switch (mc_error) {
    case MoveControlError::UNKNOWN:
      return cogrob::navigation::UNKNOWN_FAIL_REASON;
    case MoveControlError::OK:
      LOG(FATAL) << "Converting MoveControlError::OK to fail reason.";
      return cogrob::navigation::UNKNOWN_FAIL_REASON;
    case MoveControlError::CANCELLED:
      return cogrob::navigation::CANCELLED;
    case MoveControlError::UNDOCK_FAILURE:
      return cogrob::navigation::UNDOCK_ERROR;
    case MoveControlError::DOCK_FAILURE:
      return cogrob::navigation::DOCK_ERROR;
    case MoveControlError::MOVEBASE_TIMEOUT:
      return cogrob::navigation::EXECUTION_TIMEOUT;
    case MoveControlError::UNEXPECTED_ACTION_STATE:
      return cogrob::navigation::EXECUTION_FAILURE;
    case MoveControlError::TRAFFIC_CONTROL_FAIL:
      return cogrob::navigation::TRAFFIC_CONTROL_ERROR;
    case MoveControlError::GET_ROBOT_POSITION_FAIL:
      return cogrob::navigation::GET_POSITION_ERROR;
  }
}

}  // namespace

MoveControl::MoveControl(ros::NodeHandle* ros_node) {
  move_base_ = absl::make_unique<actionlib::SimpleActionClient<
      move_base_msgs::MoveBaseAction>>("/move_base", true);
  move_base_->waitForServer();
  LOG(INFO) << "MoveBase action client ready.";

  robot_position_client_ = absl::make_unique<ros::ServiceClient>(
      ros_node->serviceClient<cogrob_robot_state_msgs::GetRobotPosition>(
      "/cogrob/get_robot_position"));
  LOG(INFO) << "Robot position service ready.";

  dock_control_ = absl::make_unique<DockControl>(ros_node);

  traffic_control_stub_ =
      TrafficControl::NewStub(grpc::CreateChannel(
      "localhost:7011", grpc::InsecureChannelCredentials()));

  direct_go_to_logger_ = absl::make_unique<UniversalLogger>(
      "navigation/direct_go_to");
  move_to_location_logger_ = absl::make_unique<UniversalLogger>(
      "navigation/move_to_location");
  go_home_and_dock_logger_ = absl::make_unique<UniversalLogger>(
      "navigation/go_home_and_dock");

  log_flusher_ = absl::make_unique<UniversalLoggerFlusher>();
  log_flusher_->Register(direct_go_to_logger_.get());
  log_flusher_->Register(move_to_location_logger_.get());
  log_flusher_->Register(go_home_and_dock_logger_.get());

  latest_status_ = Status::UNKNOWN;
}

Status MoveControl::ForceCancel() {
  absl::MutexLock lock(&mutex_);
  if (is_running_.load()) {
    force_stop_.store(true);
    move_thread_->join();
    move_thread_.reset(nullptr);
    force_stop_.store(false);
  } else {
    CHECK(!move_thread_);
  }
  return Status::OK;
}

Status MoveControl::GetResult() {
  absl::MutexLock lock(&mutex_);
  if (!is_running_.load() && move_thread_) {
    move_thread_->join();
    move_thread_.reset(nullptr);
  }
  absl::ReaderMutexLock latest_status_lock(&latest_status_mutex_);
  return latest_status_;
}

bool MoveControl::IsRunning() {
  absl::MutexLock lock(&mutex_);
  if (!is_running_.load() && move_thread_) {
    move_thread_->join();
    move_thread_.reset(nullptr);
  }
  return is_running_.load();
}

Status MoveControl::GoToNearCoordinate(double x, double y, double yaw,
    double tolerance, bool auto_cancel, bool wait_finish,
    MoveControlError* move_control_error) {
  LOG(INFO) << "Moving to (" << x << ", " << y << ", " << yaw << "), "
            << "tolerance is " << tolerance << ", "
            << "auto_cancel is " << auto_cancel;

  *move_control_error = MoveControlError::UNKNOWN;

  // First, undock if near home.
  StatusOr<bool> undock_result = dock_control_->UndockIfNearHome();
  if (!undock_result.ok() || !undock_result.ValueOrDie()) {
    if (undock_result.ok()) {
      LOG(ERROR) << "Undock failed: value is false";
    } else {
      LOG(ERROR) << "Undock failed: status is " << undock_result.status();
    }
    *move_control_error = MoveControlError::UNDOCK_FAILURE;
    return Status(util::error::INTERNAL, "Undock failed.");
  }

  // Then use MoveBase to move to location.
  move_base_msgs::MoveBaseGoal move_goal;
  move_goal.target_pose.pose.position.x = x;
  move_goal.target_pose.pose.position.y = y;
  move_goal.target_pose.pose.orientation.z = std::sin(yaw / 2.0);
  move_goal.target_pose.pose.orientation.w = std::cos(yaw / 2.0);
  move_goal.target_pose.header.frame_id = "map";
  move_goal.target_pose.header.stamp = ros::Time::now();

  const int MAX_MOVE_TIME = 30;
  const std::chrono::system_clock::time_point timeout_deadline =
      std::chrono::system_clock::now() + std::chrono::seconds(MAX_MOVE_TIME);
  // Starts the action.
  move_base_->sendGoal(move_goal);

  bool is_success = false;
  bool is_action_active = true;
  while (std::chrono::system_clock::now() < timeout_deadline) {
    // Force stop requested.
    if (force_stop_) {
      *move_control_error = MoveControlError::CANCELLED;
      LOG(INFO) << "Force cancelled moving to coordinate.";
      break;
    }

    // Blocks for a short period of time, and returns if action succeed.
    if (move_base_->waitForResult(ros::Duration(0.5))) {
      if (move_base_->getState()
          == actionlib::SimpleClientGoalState::SUCCEEDED) {
        LOG(INFO) << "Moving to (" << x << ", " << y << ") action finished.";
      } else {
        *move_control_error = MoveControlError::UNEXPECTED_ACTION_STATE;
        LOG(ERROR) << "Moving to (" << x << ", " << y << ") action finished "
                   << "with state: " << move_base_->getState().getText();
      }
      is_action_active = false;
    }

    // Test if current position is within the circle of tolerance.
    cogrob_robot_state_msgs::GetRobotPosition robot_position_srv;
    if (!robot_position_client_->call(robot_position_srv)) {
      LOG(ERROR) << "GetRobotPosition failed.";
      return util::Status(util::error::UNAVAILABLE, "GetRobotPosition failed.");
    }
    double current_x = robot_position_srv.response.world_x;
    double current_y = robot_position_srv.response.world_y;
    double distance_to_target = sqrt(
        pow(current_x - x, 2) + pow(current_y - y, 2));
    if (distance_to_target < tolerance) {
      LOG(INFO) << "Finished moving to (" << x << ", " << y << "), "
                << "distance to target is " << distance_to_target << ", "
                << "tolerance is " << tolerance;
      is_success = true;
      if (wait_finish) {
        if (!is_action_active) {
          break;
        }
      } else {
        break;
      }
    }

    if (!is_action_active) {
      break;
    }
  }

  if (is_success) {
    *move_control_error = MoveControlError::OK;
    if (auto_cancel) {
      move_base_->cancelGoal();
    }
    return Status::OK;
  }

  if (std::chrono::system_clock::now() > timeout_deadline) {
    *move_control_error = MoveControlError::MOVEBASE_TIMEOUT;
    move_base_->cancelGoal();
    LOG(ERROR) << "Moving to (" << x << ", " << y << ") timeout, cancelled.";
  } else if (is_action_active) {
    *move_control_error = MoveControlError::MOVEBASE_TIMEOUT;
    LOG(ERROR) << "Moving to (" << x << ", " << y << ") failed before "
               << "reaching target.";
  }
  return Status(util::error::INTERNAL, "Failed.");
}

StatusOr<std::string> MoveControl::GetMapName() {
  traffic_control::GetNavMapNameRequest nav_map_name_request;
  traffic_control::GetNavMapNameResponse nav_map_name_response;
  grpc::ClientContext context;
  grpc::Status status = traffic_control_stub_->GetNavMapName(
      &context, nav_map_name_request, &nav_map_name_response);
  if (!status.ok()) {
    LOG(ERROR) << "Traffic control failure: " << status.error_message();
    return Status(util::error::INTERNAL, status.error_message());
  }
  return nav_map_name_response.nav_map_name();
}

Status MoveControl::PopulateCurrentLocation(
    cogrob::navigation::Location* location,
    MoveControlError* move_control_error) {
  // First, get current location.
  cogrob_robot_state_msgs::GetRobotPosition robot_position_srv;
  if (!robot_position_client_->call(robot_position_srv)) {
    LOG(ERROR) << "GetRobotPosition failed.";
    *move_control_error = MoveControlError::TRAFFIC_CONTROL_FAIL;
    return util::Status(util::error::UNAVAILABLE, "GetRobotPosition failed.");
  }
  double current_x = robot_position_srv.response.world_x;
  double current_y = robot_position_srv.response.world_y;
  double current_orientation = robot_position_srv.response.orientation;
  location->mutable_world_pos()->set_x(current_x);
  location->mutable_world_pos()->set_y(current_y);
  location->mutable_world_pos()->set_orientation(current_orientation);

  // Contact TC to get nearest NavPoint.
  traffic_control::GetNearestNavPointRequest nearest_navpoint_request;
  traffic_control::GetNearestNavPointResponse nearest_navpoint_response;
  nearest_navpoint_request.mutable_world_xy()->set_x(current_x);
  nearest_navpoint_request.mutable_world_xy()->set_y(current_y);
  grpc::ClientContext context;
  grpc::Status nearest_nav_point_status =
      traffic_control_stub_->GetNearestNavPoint(&context,
      nearest_navpoint_request, &nearest_navpoint_response);
  if (!nearest_nav_point_status.ok()) {
    LOG(ERROR) << "Traffic control failure: "
               << nearest_nav_point_status.error_message();
    *move_control_error = MoveControlError::TRAFFIC_CONTROL_FAIL;
    return Status(util::error::INTERNAL,
                  nearest_nav_point_status.error_message());
  }
  const std::string nearest_nav_point_name =
      nearest_navpoint_response.nearest_nav_point_name();

  // Populates NavPoint information.
  RETURN_IF_ERROR(PopulateNavPointInfo(nearest_nav_point_name,
      location->mutable_nearest_nav_point(), move_control_error));

  double navpoint_x = location->nearest_nav_point().world_xy().x();
  double navpoint_y = location->nearest_nav_point().world_xy().y();
  double distance_to_navpoint = sqrt(
      pow(current_x - navpoint_x, 2) + pow(current_y - navpoint_y, 2));
  location->set_distance_nearest_nav_point(distance_to_navpoint);

  return Status::OK;
}

Status MoveControl::PopulateNavPointInfo(
    const std::string& nav_point_name, cogrob::navigation::NavPoint* nav_point,
    MoveControlError* move_control_error) {
  // Contact TC to get NavPoint info.
  traffic_control::GetNavPointInfoRequest navpoint_info_request;
  traffic_control::GetNavPointInfoResponse navpoint_info_response;
  navpoint_info_request.set_nav_point_name(nav_point_name);
  grpc::ClientContext context;
  grpc::Status navpoint_info_status =
      traffic_control_stub_->GetNavPointInfo(
      &context, navpoint_info_request, &navpoint_info_response);
  if (!navpoint_info_status.ok()) {
    LOG(ERROR) << "Traffic control failure: "
               << navpoint_info_status.error_message();
    *move_control_error = MoveControlError::TRAFFIC_CONTROL_FAIL;
    return Status(util::error::INTERNAL, navpoint_info_status.error_message());
  }
  nav_point->CopyFrom(navpoint_info_response.nav_point());
  return Status::OK;
}

Status MoveControl::DirectGoTo(const std::string& nav_point,
    bool use_nav_point_yaw, double tolerance, bool auto_cancel,
    bool wait_finish, MoveControlError* move_control_error) {
  EventEntryHelper event_entry(direct_go_to_logger_.get());
  auto* direct_go_to_log =
      event_entry.GetMutableLogEntry()->mutable_direct_go_to();

  // Populates map name field in the log.
  StatusOr<std::string> map_name_status = GetMapName();
  if (!map_name_status.ok()) {
    direct_go_to_log->add_fail_reason(
        cogrob::navigation::TRAFFIC_CONTROL_MAP_NAME_ERROR);
    LOG(ERROR) << "Cannot get map name from TrafficControl, "
               << map_name_status.status();
  } else {
    direct_go_to_log->set_map_name(map_name_status.ValueOrDie());
  }

  // Populates start_point.
  Status log_location_status = PopulateCurrentLocation(
      direct_go_to_log->mutable_start_point(), move_control_error);
  if (!log_location_status.ok()) {
    direct_go_to_log->add_fail_reason(
        MoveControlErrorToFailReason(*move_control_error));
  }

  // Populates target_nav_point.
  Status log_target_nav_point = PopulateNavPointInfo(
      nav_point, direct_go_to_log->mutable_target_nav_point(),
      move_control_error);
  if (!log_target_nav_point.ok()) {
    direct_go_to_log->add_fail_reason(
        MoveControlErrorToFailReason(*move_control_error));
  }

  cogrob::navigation::NavPoint target_nav_point;
  Status nav_point_info_status =
      PopulateNavPointInfo(nav_point, &target_nav_point, move_control_error);
  if (!nav_point_info_status.ok()) {
    direct_go_to_log->add_fail_reason(
        MoveControlErrorToFailReason(*move_control_error));
    direct_go_to_log->set_result(cogrob::navigation::FAILED_RESULT);
    return nav_point_info_status;
  }
  const double nav_point_x = target_nav_point.world_xy().x();
  const double nav_point_y = target_nav_point.world_xy().y();

  double yaw = 0;

  if (use_nav_point_yaw) {
    if (target_nav_point.world_xy().orientation_valid()) {
      yaw = target_nav_point.world_xy().orientation();
    } else {
      LOG(ERROR) << "use_nav_point_yaw is enable, "
                 << "but the target nav point does not have a valid yaw value.";
      direct_go_to_log->add_fail_reason(
          cogrob::navigation::TRAFFIC_CONTROL_ERROR);
    }
  } else {
    cogrob_robot_state_msgs::GetRobotPosition robot_position_srv;
    if (!robot_position_client_->call(robot_position_srv)) {
      LOG(ERROR) << "GetRobotPosition failed, can not set yaw.";
      direct_go_to_log->add_fail_reason(cogrob::navigation::GET_POSITION_ERROR);
    } else {
      double delta_x = nav_point_x - robot_position_srv.response.world_x;
      double delta_y = nav_point_y - robot_position_srv.response.world_y;
      if (delta_y != 0 || delta_x != 0) {
        yaw = std::atan2(delta_y, delta_x);
      } else {
        LOG(ERROR) << "Target is the same as the current location.";
      }
    }
  }

  Status result = GoToNearCoordinate(
      nav_point_x, nav_point_y, yaw, tolerance, auto_cancel, wait_finish,
      move_control_error);
  if (result.ok()) {
    direct_go_to_log->set_result(cogrob::navigation::SUCCESS_RESULT);
  } else {
    direct_go_to_log->add_fail_reason(
        MoveControlErrorToFailReason(*move_control_error));
    direct_go_to_log->set_result(cogrob::navigation::FAILED_RESULT);
  }
  return result;
}

Status MoveControl::MoveToLocation(
    const std::string& nav_point, MoveControlError* move_control_error) {
  EventEntryHelper event_entry(move_to_location_logger_.get());
  auto* move_to_log =
      event_entry.GetMutableLogEntry()->mutable_move_to_location();

  // Populates map name field in the log.
  StatusOr<std::string> map_name_status = GetMapName();
  if (!map_name_status.ok()) {
    move_to_log->add_fail_reason(
        cogrob::navigation::TRAFFIC_CONTROL_MAP_NAME_ERROR);
    LOG(ERROR) << "Cannot get map name from TrafficControl, "
               << map_name_status.status();
  } else {
    move_to_log->set_map_name(map_name_status.ValueOrDie());
  }

  // Populates start_point.
  Status log_location_status = PopulateCurrentLocation(
      move_to_log->mutable_start_point(), move_control_error);
  if (!log_location_status.ok()) {
    move_to_log->add_fail_reason(
        MoveControlErrorToFailReason(*move_control_error));
  }

  // Populates target_nav_point.
  Status log_target_nav_point = PopulateNavPointInfo(
      nav_point, move_to_log->mutable_target_nav_point(),
      move_control_error);
  if (!log_target_nav_point.ok()) {
    move_to_log->add_fail_reason(
        MoveControlErrorToFailReason(*move_control_error));
  }

  // Get current location.
  cogrob_robot_state_msgs::GetRobotPosition robot_position_srv;
  if (!robot_position_client_->call(robot_position_srv)) {
    LOG(ERROR) << "GetRobotPosition failed.";
    *move_control_error = MoveControlError::TRAFFIC_CONTROL_FAIL;
    move_to_log->add_fail_reason(
        cogrob::navigation::TRAFFIC_CONTROL_ERROR);
    move_to_log->set_result(cogrob::navigation::FAILED_RESULT);
    return util::Status(util::error::UNAVAILABLE, "GetRobotPosition failed.");
  }
  double current_x = robot_position_srv.response.world_x;
  double current_y = robot_position_srv.response.world_y;

  // Contact Traffic Control to get nearest NavPoint.
  traffic_control::GetNearestNavPointRequest nearest_navpoint_request;
  traffic_control::GetNearestNavPointResponse nearest_navpoint_response;
  nearest_navpoint_request.mutable_world_xy()->set_x(current_x);
  nearest_navpoint_request.mutable_world_xy()->set_y(current_y);
  grpc::ClientContext nearest_nav_point_context;
  grpc::Status nearest_nav_point_status =
      traffic_control_stub_->GetNearestNavPoint(
      &nearest_nav_point_context, nearest_navpoint_request,
      &nearest_navpoint_response);
  if (!nearest_nav_point_status.ok()) {
    LOG(ERROR) << "Traffic control failure: "
               << nearest_nav_point_status.error_message();
    *move_control_error = MoveControlError::TRAFFIC_CONTROL_FAIL;
    move_to_log->add_fail_reason(
        cogrob::navigation::TRAFFIC_CONTROL_ERROR);
    move_to_log->set_result(cogrob::navigation::FAILED_RESULT);
    return Status(util::error::INTERNAL,
                  nearest_nav_point_status.error_message());
  }
  const std::string nearest_nav_point_name =
      nearest_navpoint_response.nearest_nav_point_name();

  LOG(INFO) << "Moving from " << nearest_nav_point_name << " towards "
            << nav_point;

  // Contact Traffic Control to get a path plan (a series of NavPoints).
  traffic_control::GetNavRoutePlanRequest nav_route_plan_request;
  traffic_control::GetNavRoutePlanResponse nav_route_plan_response;
  nav_route_plan_request.set_nav_point_start(nearest_nav_point_name);
  nav_route_plan_request.set_nav_point_end(nav_point);
  grpc::ClientContext nav_route_plan_context;
  grpc::Status nav_route_plan_status =
      traffic_control_stub_->GetNavRoutePlan(
      &nav_route_plan_context, nav_route_plan_request,
      &nav_route_plan_response);
  if (!nav_route_plan_status.ok()) {
    LOG(ERROR) << "Traffic control failure: "
               << nav_route_plan_status.error_message();
    *move_control_error = MoveControlError::TRAFFIC_CONTROL_FAIL;
    move_to_log->add_fail_reason(
        cogrob::navigation::TRAFFIC_CONTROL_ERROR);
    move_to_log->set_result(cogrob::navigation::FAILED_RESULT);
    return Status(util::error::INTERNAL,
                  nearest_nav_point_status.error_message());
  }

  CHECK_GT(nav_route_plan_response.navigation_route().size(), 0);

  // Prints the Path Plan.
  const size_t navpoints_count =
      nav_route_plan_response.navigation_route_size();
  std::string nav_plan_print;
  for (size_t i = 0; i < navpoints_count; ++i) {
    if (i > 0) {
      nav_plan_print += "-";
    }
    nav_plan_print += nav_route_plan_response.navigation_route(i);
    if (!PopulateNavPointInfo(nav_route_plan_response.navigation_route(i),
        move_to_log->add_nav_path(), move_control_error).ok()) {
      *move_control_error = MoveControlError::TRAFFIC_CONTROL_FAIL;
      move_to_log->add_fail_reason(
          cogrob::navigation::TRAFFIC_CONTROL_ERROR);
    }
  }
  LOG(INFO) << "Path plan is: " << nav_plan_print;

  for (size_t i = 0; i < navpoints_count; ++i) {
    const std::string current_target =
        nav_route_plan_response.navigation_route(i);
    auto* progress_log = move_to_log->add_progress();
    if (!PopulateCurrentLocation(
        progress_log->mutable_start_point(), move_control_error).ok()) {
      move_to_log->add_fail_reason(
          MoveControlErrorToFailReason(*move_control_error));
    }
    if (!PopulateNavPointInfo(current_target, progress_log->mutable_target(),
        move_control_error).ok()) {
      move_to_log->add_fail_reason(
          MoveControlErrorToFailReason(*move_control_error));
    }

    util::PopulateCurrentTimestamp(progress_log->mutable_start_timestamp());
    Status go_to_result;
    if (i != navpoints_count - 1) {
      go_to_result = DirectGoTo(
          current_target, false, FLAGS_move_waypoint_tolerance_distance,
          false, false, move_control_error);
    } else {
      go_to_result = DirectGoTo(
          current_target, true, FLAGS_move_destination_tolerance_distance,
          FLAGS_move_destination_cancel_goal,
          FLAGS_move_destination_wait_finish, move_control_error);
    }
    util::PopulateCurrentTimestamp(progress_log->mutable_end_timestamp());
    progress_log->set_time_elapsed_secs(
        util::TimestampProtoToDouble(progress_log->end_timestamp()) -
        util::TimestampProtoToDouble(progress_log->start_timestamp()));

    if (!go_to_result.ok()) {
      LOG(ERROR) << "Execution failure moveing to " << current_target;
      move_to_log->add_fail_reason(
          MoveControlErrorToFailReason(*move_control_error));
      move_to_log->set_result(cogrob::navigation::FAILED_RESULT);
      progress_log->add_fail_reason(
          MoveControlErrorToFailReason(*move_control_error));
      progress_log->set_result(cogrob::navigation::FAILED_RESULT);
      return go_to_result;
    } else {
      LOG(INFO) << "Moved to " << current_target;
      progress_log->set_result(cogrob::navigation::SUCCESS_RESULT);
    }
  }
  LOG(INFO) << "Route plan finished successfully.";
  move_to_log->set_result(cogrob::navigation::SUCCESS_RESULT);
  return Status::OK;
}

Status MoveControl::GoHomeAndDock(MoveControlError* move_control_error) {
  EventEntryHelper event_entry(go_home_and_dock_logger_.get());
  auto* home_and_dock_log =
      event_entry.GetMutableLogEntry()->mutable_go_home_and_dock();

  // Populates start_point.
  Status log_location_status = PopulateCurrentLocation(
      home_and_dock_log->mutable_start_point(), move_control_error);
  if (!log_location_status.ok()) {
    home_and_dock_log->add_fail_reason(
        MoveControlErrorToFailReason(*move_control_error));
  }

  // First, find the NavPoint nearest to (0, 0)
  // Contact TC to get nearest NavPoint.
  traffic_control::GetNearestNavPointRequest home_navpoint_request;
  traffic_control::GetNearestNavPointResponse home_navpoint_response;
  // TODO(shengye): Adds a get home corrdinate service in TC and replace this
  // hard-coded location.
  home_navpoint_request.mutable_world_xy()->set_x(0.0);
  home_navpoint_request.mutable_world_xy()->set_y(0.0);
  grpc::ClientContext home_nav_point_context;
  grpc::Status home_nav_point_status =
      traffic_control_stub_->GetNearestNavPoint(&home_nav_point_context,
      home_navpoint_request, &home_navpoint_response);
  if (!home_nav_point_status.ok()) {
    LOG(ERROR) << "Traffic control failure: "
               << home_nav_point_status.error_message();
    *move_control_error = MoveControlError::TRAFFIC_CONTROL_FAIL;
    return Status(util::error::INTERNAL,
                  home_nav_point_status.error_message());
  }
  const std::string home_nav_point_name =
      home_navpoint_response.nearest_nav_point_name();

  // Execute MoveToLocation nav_point.
  Status move_to_home_nav_point_status = MoveToLocation(
      home_nav_point_name, move_control_error);
  if (!move_to_home_nav_point_status.ok()) {
    LOG(ERROR) << "Move To NavPoint near home failed: "
               << move_to_home_nav_point_status.error_message();
    home_and_dock_log->add_fail_reason(
        MoveControlErrorToFailReason(*move_control_error));
    home_and_dock_log->set_result(cogrob::navigation::FAILED_RESULT);
    return move_to_home_nav_point_status;
  }

  bool dock_success = false;
  // Repeated start dock and GoToNearCoordinate until finish or too many
  // retreis.
  for (int relocate_attempt = 0; relocate_attempt < 10; ++relocate_attempt) {
    home_and_dock_log->set_relocate_attempts(relocate_attempt + 1);
    // Execute GoToNearCoordinate(-0.2, 0, ...)
    Status go_to_predock_status = GoToNearCoordinate(
        -0.4, 0, 0, 0.1, false, true, move_control_error);
    if (!go_to_predock_status.ok()) {
      LOG(ERROR) << "Move To NavPoint near home failed: "
                 << go_to_predock_status.error_message();
      home_and_dock_log->add_fail_reason(
          MoveControlErrorToFailReason(*move_control_error));
      continue;
    }
    for (int retry_time = 0; retry_time < 5; ++retry_time) {
      home_and_dock_log->set_last_dock_attempts(retry_time + 1);
      home_and_dock_log->set_total_dock_attempts(
          home_and_dock_log->total_dock_attempts() + 1);
      StatusOr<bool> dock_result = dock_control_->Dock();
      if (!dock_result.ok() || !dock_result.ValueOrDie()) {
        if (dock_result.ok()) {
          LOG(ERROR) << "Dock failed: value is false";
        } else {
          LOG(ERROR) << "Dock failed: status is " << dock_result.status();
        }
        *move_control_error = MoveControlError::DOCK_FAILURE;
      } else {
        dock_success = true;
        break;
      }
    }
    if (dock_success) {
      break;
    }
  }

  if (dock_success) {
    home_and_dock_log->set_result(cogrob::navigation::SUCCESS_RESULT);
    return Status::OK;
  } else {
    return Status(util::error::INTERNAL, "Auto dock failed.");
  }
}

void MoveControl::MoveToNavPointThread(std::string nav_point) {
  is_running_.store(true);
  MoveControlError move_control_error = MoveControlError::UNKNOWN;
  Status result;
  if (nav_point == "DOCK") {
    result = GoHomeAndDock(&move_control_error);
  } else {
    result = MoveToLocation(nav_point, &move_control_error);
  }

  LOG(INFO) << "MoveThread is about to finish, result: " << result;
  absl::WriterMutexLock lock(&latest_status_mutex_);
  latest_status_ = result;
  is_running_.store(false);
}

Status MoveControl::StartMoveToNavPoint(const std::string& nav_point) {
  absl::MutexLock lock(&mutex_);
  if (is_running_.load()) {
    return Status(util::error::INTERNAL, "Another task is running.");
  }
  if (move_thread_) {
    move_thread_->join();
    move_thread_.reset(nullptr);
  }
  move_thread_ = absl::make_unique<std::thread>(
      [this, nav_point] {
        this->MoveToNavPointThread(nav_point);
      });
  return Status::OK;
}

}  // namespace cogrob_navigation_route

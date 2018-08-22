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

#include "cogrob/navigation/traffic_control/traffic_control_server.h"

#include <cmath>
#include <deque>
#include <stack>

#include "third_party/glog.h"

namespace cogrob {
namespace navigation {
namespace traffic_control {

TrafficControlServiceImpl::TrafficControlServiceImpl(
    std::unique_ptr<cogrob::navigation::NavChart> nav_chart) {
  absl::WriterMutexLock lock(&nav_chart_mutex_);

  nav_chart_ = std::move(nav_chart);

  for (const auto& nav_point : nav_chart_->nav_points()) {
    nav_points_.insert(std::make_pair(nav_point.name(), nav_point));
    nav_point_neighbors_[nav_point.name()] = {};
  }

  for (const auto& nav_path : nav_chart_->nav_paths()) {
    CHECK(nav_points_.count(nav_path.end_a()));
    CHECK(nav_points_.count(nav_path.end_b()));

    const NavPoint& point_a = nav_points_[nav_path.end_a()];
    const NavPoint& point_b = nav_points_[nav_path.end_b()];

    nav_point_neighbors_[point_a.name()].push_back(point_b.name());
    nav_point_neighbors_[point_b.name()].push_back(point_a.name());

    nav_paths_[std::make_pair(point_a.name(), point_b.name())] = nav_path;
    nav_paths_[std::make_pair(point_b.name(), point_a.name())] = nav_path;

    const double path_cost = std::sqrt(
        std::pow(point_a.world_xy().x() - point_b.world_xy().x(), 2.0) +
        std::pow(point_a.world_xy().y() - point_b.world_xy().y(), 2.0));
    nav_path_cost_[std::make_pair(point_a.name(), point_b.name())] = path_cost;
    nav_path_cost_[std::make_pair(point_b.name(), point_a.name())] = path_cost;
  }
}

grpc::Status TrafficControlServiceImpl::GetFullNavChart(
    grpc::ServerContext* context, const GetFullNavChartRequest* request,
    GetFullNavChartResponse* response) {
  absl::ReaderMutexLock lock(&nav_chart_mutex_);
  *(response->mutable_nav_chart()) = *nav_chart_;
  return grpc::Status::OK;
}

grpc::Status TrafficControlServiceImpl::ListNavPoints(
    grpc::ServerContext* context, const ListNavPointsRequest* request,
    ListNavPointsResponse* response) {
  absl::ReaderMutexLock lock(&nav_chart_mutex_);
  response->mutable_nav_points()->CopyFrom(nav_chart_->nav_points());
  return grpc::Status::OK;
}

grpc::Status TrafficControlServiceImpl::GetNavPointInfo(
    grpc::ServerContext* context, const GetNavPointInfoRequest* request,
    GetNavPointInfoResponse* response) {
  absl::ReaderMutexLock lock(&nav_chart_mutex_);
  if (!nav_points_.count(request->nav_point_name())) {
    return grpc::Status(grpc::INVALID_ARGUMENT,
                        "Cannot find NavPoint " + request->nav_point_name());
  }
  *(response->mutable_nav_point())
      = nav_points_[request->nav_point_name()];
  for (const std::string& neighbor :
      nav_point_neighbors_[request->nav_point_name()]) {
    response-> add_neighbors(neighbor);
  }
  return grpc::Status::OK;
}

grpc::Status TrafficControlServiceImpl::GetNavPathInfo(
    grpc::ServerContext* context, const GetNavPathInfoRequest* request,
    GetNavPathInfoResponse* response) {
  absl::ReaderMutexLock lock(&nav_chart_mutex_);
  if (!nav_paths_.count(std::make_pair(request->end_a(), request->end_b()))) {
    return grpc::Status(grpc::INVALID_ARGUMENT,
                        "Cannot find NavPath between " + request->end_a()
                        + " and " + request->end_b());
  }
  *(response->mutable_nav_path()) =
      nav_paths_[std::make_pair(request->end_a(), request->end_b())];
  response->set_line_distance(
      nav_path_cost_[std::make_pair(request->end_a(), request->end_b())]);
  response->set_weighted_distance(
      nav_path_cost_[std::make_pair(request->end_a(), request->end_b())]);
  return grpc::Status::OK;
}

grpc::Status TrafficControlServiceImpl::GetNearestNavPoint(
    grpc::ServerContext* context, const GetNearestNavPointRequest* request,
    GetNearestNavPointResponse* response) {
  absl::ReaderMutexLock lock(&nav_chart_mutex_);
  // TODO(shengye): If there is a performance issue, use KD-tree.
  double best_distance = std::numeric_limits<double>::infinity();
  std::string best_match_name = "???";
  for (const auto& nav_point_pair : nav_points_) {
    const auto& nav_point = nav_point_pair.second;
    const double current_distance = std::sqrt(
        std::pow(nav_point.world_xy().x() - request->world_xy().x(), 2.0) +
        std::pow(nav_point.world_xy().y() - request->world_xy().y(), 2.0));
    if (current_distance < best_distance) {
      best_distance = current_distance;
      best_match_name = nav_point.name();
    }
  }
  response->set_nearest_nav_point_name(best_match_name);
  return grpc::Status::OK;
}

grpc::Status TrafficControlServiceImpl::GetNavMapName(
    grpc::ServerContext* context, const GetNavMapNameRequest* request,
    GetNavMapNameResponse* response) {
  absl::ReaderMutexLock lock(&nav_chart_mutex_);
  response->set_nav_map_name(nav_chart_->grid_map().map_name());
  return grpc::Status::OK;
}

grpc::Status TrafficControlServiceImpl::GetNavRoutePlan(
    grpc::ServerContext* context, const GetNavRoutePlanRequest* request,
    GetNavRoutePlanResponse* response) {
  absl::ReaderMutexLock lock(&nav_chart_mutex_);

  if (!nav_points_.count(request->nav_point_start())
      || !nav_points_.count(request->nav_point_end())) {
    return grpc::Status(grpc::INVALID_ARGUMENT,
                        "Cannot find NavPoint " + request->ShortDebugString());
  }

  // TODO(shengye): If performance is an issue, cache the result.

  // The SPFA algorithm.
  std::unordered_map<std::string, double> distances;
  std::unordered_map<std::string, std::string> best_from;
  std::deque<std::string> spfa_queue;
  std::unordered_set<std::string> in_queue;
  in_queue.insert(request->nav_point_start());
  spfa_queue.push_back(request->nav_point_start());
  distances[request->nav_point_start()] = 0;
  best_from[request->nav_point_start()] = "???";

  while (!spfa_queue.empty()) {
    std::string current = spfa_queue.front();
    spfa_queue.pop_front();
    in_queue.erase(current);
    for (const std::string& neighbor : nav_point_neighbors_[current]) {
      double new_distance = distances[current]
          + nav_path_cost_[std::make_pair(current, neighbor)];
      if ((distances.count(neighbor) == 0)
          || new_distance < distances[neighbor]) {
        distances[neighbor] = new_distance;
        best_from[neighbor] = current;
        if (!in_queue.count(neighbor)) {
          in_queue.insert(neighbor);
          spfa_queue.push_back(neighbor);
        }
      }
    }
  }

  if (!distances.count(request->nav_point_end())) {
    return grpc::Status(grpc::NOT_FOUND,
                        "Cannot find a path: " + request->ShortDebugString());
  }

  std::stack<std::string> result;
  std::string current = request->nav_point_end();
  while (current != request->nav_point_start()) {
    result.push(current);
    current = best_from[current];
  }
  result.push(request->nav_point_start());
  while (!result.empty()) {
    response->add_navigation_route(result.top());
    result.pop();
  }

  return grpc::Status::OK;
}

grpc::Status TrafficControlServiceImpl::ReportNavFailure(
    grpc::ServerContext* context, const ReportNavFailureRequest* request,
    ReportNavFailureResponse* response) {
  absl::ReaderMutexLock lock(&nav_chart_mutex_);
  LOG(ERROR) << "Reported failure: " << request->ShortDebugString();
  return grpc::Status::OK;
}

}  // namespace traffic_control
}  // namespace navigation
}  // namespace cogrob

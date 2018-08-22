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

#ifndef COGROB_NAVIGATION_TRAFFIC_CONTROL_TRAFFIC_CONTROL_SERVER_H_
#define COGROB_NAVIGATION_TRAFFIC_CONTROL_TRAFFIC_CONTROL_SERVER_H_

#include <grpc++/grpc++.h>
#include <unordered_map>
#include <memory.h>

#include "absl/synchronization/mutex.h"
#include "cogrob/navigation/proto/navigation_chart.pb.h"
#include "cogrob/navigation/traffic_control/proto/traffic_control.grpc.pb.h"
#include "util/hash.h"

namespace cogrob {
namespace navigation {
namespace traffic_control {

class TrafficControlServiceImpl final : public TrafficControl::Service {
 public:
  TrafficControlServiceImpl(
      std::unique_ptr<cogrob::navigation::NavChart> nav_chart);

  grpc::Status GetFullNavChart(
      grpc::ServerContext* context, const GetFullNavChartRequest* request,
      GetFullNavChartResponse* response) override;
  grpc::Status ListNavPoints(
      grpc::ServerContext* context, const ListNavPointsRequest* request,
      ListNavPointsResponse* response) override;
  grpc::Status GetNavPointInfo(
      grpc::ServerContext* context, const GetNavPointInfoRequest* request,
      GetNavPointInfoResponse* response) override;
  grpc::Status GetNavPathInfo(
      grpc::ServerContext* context, const GetNavPathInfoRequest* request,
      GetNavPathInfoResponse* response) override;
  grpc::Status GetNearestNavPoint(
      grpc::ServerContext* context, const GetNearestNavPointRequest* request,
      GetNearestNavPointResponse* response) override;
  grpc::Status GetNavMapName(
      grpc::ServerContext* context, const GetNavMapNameRequest* request,
      GetNavMapNameResponse* response) override;

  grpc::Status GetNavRoutePlan(
      grpc::ServerContext* context, const GetNavRoutePlanRequest* request,
      GetNavRoutePlanResponse* response) override;
  grpc::Status ReportNavFailure(
      grpc::ServerContext* context, const ReportNavFailureRequest* request,
      ReportNavFailureResponse* response) override;

 private:
  absl::Mutex nav_chart_mutex_;
  std::unique_ptr<cogrob::navigation::NavChart>
      nav_chart_ GUARDED_BY(nav_chart_mutex_);
  std::unordered_map<std::string, NavPoint>
      nav_points_ GUARDED_BY(nav_chart_mutex_);
  std::unordered_map<std::string, std::vector<std::string>>
      nav_point_neighbors_ GUARDED_BY(nav_chart_mutex_);
  std::unordered_map<std::pair<std::string, std::string>, NavPath,
      ::util::pair_hash> nav_paths_ GUARDED_BY(nav_chart_mutex_);
  std::unordered_map<std::pair<std::string, std::string>, double,
      ::util::pair_hash> nav_path_cost_ GUARDED_BY(nav_chart_mutex_);
};

}  // namespace traffic_control
}  // namespace navigation
}  // namespace cogrob

#endif  // COGROB_NAVIGATION_TRAFFIC_CONTROL_TRAFFIC_CONTROL_SERVER_H_

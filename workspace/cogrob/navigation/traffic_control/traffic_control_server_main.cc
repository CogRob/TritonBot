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

#include <fstream>
#include <memory>
#include <string>

#include "absl/memory/memory.h"
#include "cogrob/navigation/traffic_control/traffic_control_server.h"
#include "third_party/gflags.h"
#include "third_party/glog.h"

using cogrob::navigation::traffic_control::TrafficControlServiceImpl;
using cogrob::navigation::NavChart;

DEFINE_uint64(port, 7011, "Port of the Traffic Control RPC server");
DEFINE_string(nav_chart_pb, "/home/cogrob_data/model/nav_chart.pb",
              "Path to the NavChart binary protobuf.");

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // First, load the NavChart.
  // TODO(shengye): Use FileInputStream if that could be better.
  std::fstream input_stream(FLAGS_nav_chart_pb,
                            std::ios::in | std::ios::binary);
  auto nav_chart = absl::make_unique<NavChart>();
  CHECK(nav_chart->ParseFromIstream(&input_stream));
  input_stream.close();

  // Then, build and start the server
  std::string server_address = "0.0.0.0:" + std::to_string(FLAGS_port);

  TrafficControlServiceImpl service(std::move(nav_chart));
  grpc::ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());

  LOG(INFO) << "Traffic Control RPC server listening on " << server_address;
  server->Wait();
}

// Copyright 2026 Nobleo Autonomous Solutions B.V.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <memory>
#include <unordered_map>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using namespace std::chrono_literals;

class DeferredService : public rclcpp::Node
{
public:
  DeferredService()
  : Node("deferred_service")
  {
    service_ = create_service<AddTwoInts>(
      "add_two_ints",
      [this](
        std::shared_ptr<Service> service,
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<AddTwoInts::Request> request)
      {
        handle_request(service, request_header, request);
      });
  }

private:
  using Service = rclcpp::Service<AddTwoInts>;

  void
  handle_request(
    std::shared_ptr<Service> service,
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<AddTwoInts::Request> request)
  {
    RCLCPP_INFO(
      get_logger(), "Received request: %" PRId64 " + %" PRId64,
      request->a, request->b);

    AddTwoInts::Response response;
    response.sum = request->a + request->b;
    const auto timer_id = next_timer_id_++;
    auto timer = create_wall_timer(
      5s,
      [this, service, request_header, response, timer_id]
      {
        // Unfortunately send_response wants a mutable reference
        auto response_to_send = response;
        service->send_response(*request_header, response_to_send);
        RCLCPP_INFO(get_logger(), "Sent deferred response: %" PRId64, response.sum);
        timers_.erase(timer_id);
      });
    timers_.emplace(timer_id, std::move(timer));
  }

  Service::SharedPtr service_;
  std::unordered_map<std::uint64_t, rclcpp::TimerBase::SharedPtr> timers_;
  std::uint64_t next_timer_id_{0};
};

int
main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeferredService>());
  rclcpp::shutdown();
  return 0;
}

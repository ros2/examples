// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/int32.hpp>

class AsyncReceiveCallbackClient : public rclcpp::Node
{
public:
  AsyncReceiveCallbackClient()
  : Node("examples_rclcpp_async_recv_cb_client")
  {
    // Create AddTwoInts client
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    // Wait until service is avaible
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service is not available, trying again after 1 second");
    }

    // Create a subcription to an input topic
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "input_topic", 10,
      std::bind(&AsyncReceiveCallbackClient::topic_callback, this, std::placeholders::_1));

    // Create a publisher for broadcasting the result
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("output_topic", 10);

    RCLCPP_INFO(this->get_logger(), "DelayedSumClient Initialized.");
  }

private:
  void topic_callback(const std::shared_ptr<std_msgs::msg::Int32> msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received %d at topic.", msg->data);
    if (msg->data >= 0) {
      RCLCPP_INFO(this->get_logger(), "  Input topic is %d >= 0. Requesting sum...", msg->data);

      // Create request to sum msg->data + 100
      auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
      request->a   = msg->data;
      request->b   = 100;

      // Calls the service and bind the callback to receive response (not blocking!)
      auto future_result = client_->async_send_request(
        request,
        std::bind(
          &AsyncReceiveCallbackClient::handle_service_response, this, std::placeholders::_1));
    } else {
      RCLCPP_INFO(this->get_logger(), "  Input topic is %d < 0. No request is sent", msg->data);
    }
  }

  // Callback to receive response (call inside the spinning method like any other callback)
  void handle_service_response(
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response: %ld", response->sum);

    // Publish response at output topic
    auto result_msg = std_msgs::msg::Int32();
    result_msg.data = response->sum;
    publisher_->publish(result_msg);
  }

  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsyncReceiveCallbackClient>());
  rclcpp::shutdown();
  return 0;
}

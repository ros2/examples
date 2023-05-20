#include <chrono>
#include <random>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class MultiThreadMutuallyExclusiveSubscriber : public rclcpp::Node {

public:
  MultiThreadMutuallyExclusiveSubscriber(rclcpp::NodeOptions node_options)
      : Node("subscriber_node",
             node_options.allow_undeclared_parameters(true)) {
    rclcpp::SubscriptionOptions options;
    options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    short_subscriber_ = create_subscription<std_msgs::msg::String>(
        "/short_topic", rclcpp::QoS(10),
        std::bind(&MultiThreadMutuallyExclusiveSubscriber::ShortTopicCallback,
                  this, std::placeholders::_1),
        options);

    options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    long_subscriber_ = create_subscription<std_msgs::msg::String>(
        "/long_topic", rclcpp::QoS(10),
        std::bind(&MultiThreadMutuallyExclusiveSubscriber::LongTopicCallback,
                  this, std::placeholders::_1),
        options);

    timer_ = create_wall_timer(
        1s, std::bind(&MultiThreadMutuallyExclusiveSubscriber::TimerCallback,
                      this));

    auto param_change_callback =
        [this](std::vector<rclcpp::Parameter> parameters) {
          for (auto parameter : parameters) {
            if (parameter.get_name() == "use_mutex") {
              use_mutex = parameter.as_bool();
              if (use_mutex)
                RCLCPP_INFO(get_logger(), "will use mutex");
              else
                RCLCPP_INFO(get_logger(), "will not mutex");
            }
          }

          auto result = rcl_interfaces::msg::SetParametersResult();
          result.successful = true;
          return result;
        };

    callback_handler =
        this->add_on_set_parameters_callback(param_change_callback);
  }

private:
  bool use_mutex{false};
  struct {
    std::mutex short_topic_mutex;
    std::mutex long_topic_mutex;
  } mutex_list;
  std::string processed_short_string_;
  std::string processed_long_string_;
  void ProcessString(const std::string &raw_string, std::string &output);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr short_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr long_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr callback_handler;

  void ShortTopicCallback(const std_msgs::msg::String::SharedPtr msg);
  void LongTopicCallback(const std_msgs::msg::String::SharedPtr msg);
  void TimerCallback();
};

void MultiThreadMutuallyExclusiveSubscriber::ProcessString(
    const std::string &raw_string, std::string &output) {
  output = "** ";

  for (char c : raw_string) {
    output.push_back(c);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  output += " **";
};

void MultiThreadMutuallyExclusiveSubscriber::ShortTopicCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  std::shared_ptr<std::unique_lock<std::mutex>> lock;
  if (use_mutex)
    mutex_list.short_topic_mutex.lock();
  ProcessString(msg->data, processed_short_string_);
  RCLCPP_INFO(get_logger(), "Setting processed:  %s",
              processed_short_string_.c_str());
  if (use_mutex)
    mutex_list.short_topic_mutex.unlock();
}

void MultiThreadMutuallyExclusiveSubscriber::LongTopicCallback(
    const std_msgs::msg::String::SharedPtr msg) {

  if (use_mutex)
    mutex_list.long_topic_mutex.lock();
  ProcessString(msg->data, processed_long_string_);
  RCLCPP_INFO(get_logger(), "Setting processed:  %s",
              processed_long_string_.c_str());
  if (use_mutex)
    mutex_list.long_topic_mutex.unlock();
}

void MultiThreadMutuallyExclusiveSubscriber::TimerCallback() {
  if (use_mutex) {
    mutex_list.short_topic_mutex.lock();
    mutex_list.long_topic_mutex.lock();
  }
  RCLCPP_INFO(get_logger(),
              "Getting processed strings: \n  [long] %s \n [short] %s",
              processed_long_string_.c_str(), processed_short_string_.c_str());

  if (use_mutex) {
    mutex_list.short_topic_mutex.unlock();
    mutex_list.long_topic_mutex.unlock();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions option;
  auto node = std::make_shared<MultiThreadMutuallyExclusiveSubscriber>(option);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <random>

class RandomPublisher : public rclcpp::Node
{
public:
  RandomPublisher()
      : Node("random_publisher"), count_(0)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    pub1_ = this->create_publisher<std_msgs::msg::String>("topic_A", qos);
    pub2_ = this->create_publisher<std_msgs::msg::String>("topic_B", qos);
    pub3_ = this->create_publisher<std_msgs::msg::String>("topic_C", qos);

    auto run_random_publish = [this]() {
      std_msgs::msg::String msg1, msg2, msg3;
      std::vector<std::function<void()>> publishers;
      msg1.data = "A";
      msg2.data = "B";
      msg3.data = "C";

      publishers.emplace_back(([&](){pub1_->publish(msg1);}));
      publishers.emplace_back(([&](){pub2_->publish(msg2);}));
      publishers.emplace_back(([&](){pub3_->publish(msg3);}));
      unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
      std::default_random_engine e(seed);
      std::vector<int> msgs_indexes{0, 1, 2};
      std::vector<std::string> msgs_data{"A", "B", "C"};

      while (rclcpp::ok()) {
        std::shuffle (msgs_indexes.begin(), msgs_indexes.end(), e);
        RCLCPP_INFO(this->get_logger(), "%s%s%s",
                    msgs_data[msgs_indexes.at(0)].c_str(),
                    msgs_data[msgs_indexes.at(1)].c_str(),
                    msgs_data[msgs_indexes.at(2)].c_str());
        publishers.at(msgs_indexes.at(0))();
        publishers.at(msgs_indexes.at(1))();
        publishers.at(msgs_indexes.at(2))();

        ++count_;
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
      }
    };

    thread_ = std::thread(run_random_publish);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub3_;
  std::thread thread_;
  size_t count_;
};


int32_t main(const int32_t argc, char ** const argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("waitset_node");
  auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

  auto sub1 = node->create_subscription<std_msgs::msg::String>("topic_A", qos, do_nothing);
  auto sub2 = node->create_subscription<std_msgs::msg::String>("topic_B", qos, do_nothing);
  auto sub3 = node->create_subscription<std_msgs::msg::String>("topic_C", qos, do_nothing);

  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(sub1);
  wait_set.add_subscription(sub2);
  wait_set.add_subscription(sub3);

  RandomPublisher publisher_node;

  auto num_recv = std::size_t();
  while (rclcpp::ok()) {
    const auto wait_result = wait_set.wait(std::chrono::seconds(5));

    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      bool expected_sub_triggered =
        wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U] &&
        wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1U] &&
        wait_result.get_wait_set().get_rcl_wait_set().subscriptions[2U];

      if (expected_sub_triggered) {
        std_msgs::msg::String msg1;
        std_msgs::msg::String msg2;
        std_msgs::msg::String msg3;
        msg1.data.clear();
        msg2.data.clear();
        msg3.data.clear();
        rclcpp::MessageInfo msg_info;
        if (sub1->take(msg1, msg_info)) {
          ++num_recv;
        }
        if (sub2->take(msg2, msg_info)) {
          ++num_recv;
        }
        if (sub3->take(msg3, msg_info)) {
          ++num_recv;
        }
        // handle topics A B C all together
        RCLCPP_INFO(
            node->get_logger(), "%s%s%s",
            msg1.data.c_str(), msg2.data.c_str(), msg3.data.c_str());
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      std::cerr << "No message received after 5s." << std::endl;
    } else {
      std::cerr << "Waitset failed" << std::endl;
    }
  }

  rclcpp::shutdown();
  return 0;
}

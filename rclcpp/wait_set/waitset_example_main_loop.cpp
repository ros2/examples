#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <random>

int32_t main(const int32_t argc, char ** const argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("waitset_node");
  auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

  auto sub1 = node->create_subscription<std_msgs::msg::String>("a11", qos, do_nothing);
  auto sub2 = node->create_subscription<std_msgs::msg::String>("a22", qos, do_nothing);
  auto sub3 = node->create_subscription<std_msgs::msg::String>("a33", qos, do_nothing);

  std_msgs::msg::String msg1;
  // msg1.data = "Hello";
  msg1.data = "A";

  std_msgs::msg::String msg2;
  // msg2.data = "world";
  msg2.data = "B";

  std_msgs::msg::String msg3;
  // msg3.data = "from waitset";
  msg3.data = "C";

  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(sub1);
  wait_set.add_subscription(sub2);
  wait_set.add_subscription(sub3);

  rclcpp::Node publisher_node("publisher_node");
  const auto pub1 =
    publisher_node.create_publisher<std_msgs::msg::String>("a11", qos);
  const auto pub2 =
    publisher_node.create_publisher<std_msgs::msg::String>("a22", qos);
  const auto pub3 =
    publisher_node.create_publisher<std_msgs::msg::String>("a33", qos);

  std::vector<std::function<void()>> random_publisher;
  random_publisher.emplace_back(([&](){pub1->publish(msg1);}));
  random_publisher.emplace_back(([&](){pub2->publish(msg2);}));
  random_publisher.emplace_back(([&](){pub3->publish(msg3);}));
  auto thread = std::thread([&random_publisher, &publisher_node]() {
    auto count{0};
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine e(seed);
    std::vector<int> indexes{0, 1, 2};
    std::vector<std::string> msgs{"A", "B", "C"};
    while (rclcpp::ok()) {
      std::shuffle (indexes.begin(), indexes.end(), e);
      RCLCPP_INFO(publisher_node.get_logger(), "%s%s%s",
                  msgs[indexes.at(0)].c_str(),
                  msgs[indexes.at(1)].c_str(),
                  msgs[indexes.at(2)].c_str());
      random_publisher.at(indexes.at(0))();
      random_publisher.at(indexes.at(1))();
      random_publisher.at(indexes.at(2))();
      ++count;
      std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    }
  });
  /*
  auto thread = std::thread(
      [msg1, pub1, msg2, pub2, msg3, pub3, &publisher_node]() {

        auto count{0};
        while (rclcpp::ok()) {

          // optional: make publish order random
          // i.e:
          // ABC, CBA, BAC, BCA, ....
          // if different rates are used:
          // AB, CBA, BAC, BCA, ....
          // currently is always: ACB, ACB
          pub1->publish(msg1);
          RCLCPP_INFO(publisher_node.get_logger(), "%s", msg1.data.c_str());

          if (count % 2 == 0) {
            if ((count/2) % 2 == 0) {
              pub3->publish(msg3);
              RCLCPP_INFO(publisher_node.get_logger(), "%s", msg3.data.c_str());
            }

            pub2->publish(msg2);
            RCLCPP_INFO(publisher_node.get_logger(), "%s", msg2.data.c_str());
          }
          ++count;
          std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        }
      });
      */
/*
  auto thread1 = std::thread(
    [msg1, pub1, &publisher_node]() {
      while (rclcpp::ok()) {

        pub1->publish(msg1);
        // std::cout << "Publishing msg1: " << msg1.data << std::endl;
        RCLCPP_INFO(publisher_node.get_logger(), "%s", msg1.data.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
      }
    });

  auto thread2 = std::thread(
    [msg2, pub2, &publisher_node]() {
      while (rclcpp::ok()) {
        pub2->publish(msg2);
        // std::cout << "Publishing msg2: " << msg2.data << std::endl;
        RCLCPP_INFO(publisher_node.get_logger(), "%s", msg2.data.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      }
    });

  auto thread3 = std::thread(
    [msg3, pub3, &publisher_node]() {
      while (rclcpp::ok()) {
        pub3->publish(msg3);
        // std::cout << "Publishing msg3: " << msg3.data << std::endl;
        RCLCPP_INFO(publisher_node.get_logger(), "%s", msg3.data.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
      }
    });
*/

  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  auto num_recv = std::size_t();
  while (rclcpp::ok()) {
    // Waiting up to 5s for a sample to arrive.
    const auto wait_result = wait_set.wait(std::chrono::seconds(5));

    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      // Use sub1 as triggering condition for all subs?

      bool expected_sub_triggered =
        wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U] &&
        wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1U];

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
          RCLCPP_INFO(
              node->get_logger(), "%s%s%s",
              msg1.data.c_str(), msg2.data.c_str(), msg3.data.c_str());
        } else {
          RCLCPP_INFO(
              node->get_logger(), "%s%s",
              msg1.data.c_str(), msg2.data.c_str());
        }
        // std::cout << "I heard: " << msg1.data << msg2.data << msg3.data << std::endl;
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      std::cerr << "No message received after 5s." << std::endl;
    } else {
      std::cerr << "Waitset failed" << std::endl;
    }
  }

  rclcpp::shutdown();

  thread.join();
  /*
  thread1.join();
  thread2.join();
  thread3.join();
  */
  return 0;
}

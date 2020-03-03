// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <memory>
#include <fstream>
std::fstream myfile;
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
       MinimalPublisher()
           : Node("minimal_publisher"), count_(0)
       {
                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", enter minimal_publisher"
                            << "\n";
                     myfile.close();
                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", enter publisher"
                            << "\n";
                     myfile.close();

              publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
                    
                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", exit publisher"
                            << "\n";
                     myfile.close();
                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", enter timer declaration"
                            << "\n";
                     myfile.close();

              timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));

                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", exit timer declaration"
                            << "\n";
                     myfile.close();
                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);

                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", exit minimal_publisher"
                            << "\n";
                     myfile.close();
       }

private:
       void timer_callback()
       {
                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", enter timer callback"
                            << "\n";
                     myfile.close();

              auto message = std_msgs::msg::String();
              message.data = "Hello, world! " + std::to_string(count_++);
              RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                    
                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", enter publishing"
                            << "\n";
                     myfile.close();

              publisher_->publish(message);

                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", exit publishing"
                            << "\n";
                     myfile.close();
                     myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
                     myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", exit timer callback"
                            << "\n";
                     myfile.close();
       }
       rclcpp::TimerBase::SharedPtr timer_;
       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
       size_t count_;
};

int main(int argc, char *argv[])
{
              myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
              myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", enter init"
                     << "\n";
              myfile.close();

       rclcpp::init(argc, argv);

              myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
              myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", exit init"
                     << "\n";
              myfile.close();
              myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
              myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", enter spin"
                     << "\n";
              myfile.close();

       rclcpp::spin(std::make_shared<MinimalPublisher>());

              myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
              myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", exit spin"
                     << "\n";
              myfile.close();
              myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
              myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", enter shutdown"
                     << "\n";
              myfile.close();

       rclcpp::shutdown();
       
              myfile.open("example.txt", std::fstream::in | std::fstream::out | std::fstream::app);
              myfile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ", exit shutdown"
                     << "\n";
              myfile.close();
       return 0;
}

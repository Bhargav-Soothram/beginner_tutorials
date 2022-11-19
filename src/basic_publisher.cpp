/**
 * Copyright (c) 2022 Bhargav Kumar Soothram
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file basic_publisher.cpp
 * @author Bhargav Kumar Soothram (bsoothra@umd.edu)
 * @brief Publishes string messages to a given topic
 * @version 0.1
 * @date 2022-11-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <basic_publisher.hpp>

MinimalPublisher::MinimalPublisher(const std::string &node_name,
                                   std::string topic_name)
    : Node(node_name) {
  
  // Declaring parameters
  this->declare_parameter("my_message", "Stranger Things!");
  this->declare_parameter("my_message_freq", 1000);

  // Reading from parameters
  message_.data = this->get_parameter("my_message").as_string();
  int message_freq = this->get_parameter("my_message_freq").as_int();

  // For covering all the five log levels
  if (message_freq < 500) {
    RCLCPP_FATAL(this->get_logger(),
                "Too quick to read, aborting...");
  exit(2);
  }
  else if (!(message_freq < 500) && (message_freq < 700)) {
    RCLCPP_ERROR(this->get_logger(), "Might face difficulty reading at such high message publish rates!");
  }
  else {
    RCLCPP_DEBUG(this->get_logger(), "Starting the publisher...");
  }

  // creating callback to node
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(message_freq),
      std::bind(&MinimalPublisher::timer_callback, this));
  
  // creating a publisher node
  publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
  service_ = this->create_service<cpp_pubsub::srv::ModifyString>("modify_string", std::bind(&MinimalPublisher::update_string, this, std::placeholders::_1, std::placeholders::_2));
}

// Publishing to a topic
void MinimalPublisher::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
  publisher_->publish(message_);
}

// Processing requests
void MinimalPublisher::update_string(const std::shared_ptr<cpp_pubsub::srv::ModifyString::Request> request,
          std::shared_ptr<cpp_pubsub::srv::ModifyString::Response> response) {
            message_.data = request->new_string;
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Incoming request\nnew_string: %s", request->new_string.c_str());
            response->status = "STRING CHANGED!";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", (long int)response->status.c_str());
          }

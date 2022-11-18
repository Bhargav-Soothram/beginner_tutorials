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
                                   std::string topic_name,
                                   std::string pub_msg,
                                   int time_intl)
    : Node(node_name) {
  message_.data = "Stranger Things!";
  publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(time_intl),
      std::bind(&MinimalPublisher::timer_callback, this));
  service_ = this->create_service<cpp_pubsub::srv::ModifyString>("modify_string", std::bind(&MinimalPublisher::change_string, this, std::placeholders::_1, std::placeholders::_2));
}

void MinimalPublisher::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
  publisher_->publish(message_);
}

void MinimalPublisher::change_string(const std::shared_ptr<cpp_pubsub::srv::ModifyString::Request> request,
          std::shared_ptr<cpp_pubsub::srv::ModifyString::Response> response) {
            message_.data = request->new_string;
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Incoming request\nnew_string: %s", request->new_string.c_str());
            response->status = "STRING CHANGED!";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", (long int)response->status.c_str());
          }

/**
 * Copyright (c) 2022 Bhargav Kumar Soothram
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
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
 * @file basic_subscriber.cpp
 * @author Bhargav Kumar Soothram (bsoothra@umd.edu)
 * @brief Subscribes to string messages from a given topic
 * @version 0.1
 * @date 2022-11-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <basic_subscriber.hpp>

MinimalSubscriber::MinimalSubscriber(const std::string &node_name, std::string topic_name) : 
  Node(node_name) {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
        topic_name, 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
}

void MinimalSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg) const{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

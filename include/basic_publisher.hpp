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
 * @file basic_publisher.hpp
 * @author Bhargav Kumar Soothram (bsoothra@umd.edu)
 * @brief Header file for basic_publisher.cpp
 * @version 0.1
 * @date 2022-11-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef INCLUDE_BASIC_PUBLISHER_HPP_
#define INCLUDE_BASIC_PUBLISHER_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "cpp_pubsub/srv/modify_string.hpp"

class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   * 
   * @param node_name Name of the publisher node
   * @param topic_name Name of the topic over which messages are sent
   */
  MinimalPublisher(const std::string &node_name = "minimal_publisher",
                   std::string topic_name = "chatter");

 private:
  rclcpp::TimerBase::SharedPtr timer_;  //!< Pointer to callback 
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  //!< Pointer to publisher  
  std_msgs::msg::String message_;  //!< Message packet from a topic
  rclcpp::Service<cpp_pubsub::srv::ModifyString>::SharedPtr service_;  //!< Pointer to service

  /**
   * @brief Publishes messages and prints publish messages
   * 
   */
  void timer_callback();

  /**
   * @brief Modifies the string printed upon a request 
   * 
   * @param request request message from the client
   * @param response response sent by the server
   */
  void update_string(const std::shared_ptr<cpp_pubsub::srv::ModifyString::Request> request,
          std::shared_ptr<cpp_pubsub::srv::ModifyString::Response> response);
};

#endif  // INCLUDE_BASIC_PUBLISHER_HPP_

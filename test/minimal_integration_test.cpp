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
 * @brief Integration test for a service call
 * @version 0.1
 * @date 2022-11-27
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "cpp_pubsub/srv/modify_string.hpp"

namespace integration_test_alpha {
class TestingFixture : public testing::Test {
 public:
  TestingFixture() : node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup things that should occur before every test instance should go here
    client_ =
        node_->create_client<cpp_pubsub::srv::ModifyString>("modify_string");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Setup Complete");
  }

  std::string send_service_call() {
    auto request = std::make_shared<cpp_pubsub::srv::ModifyString::Request>();
    request->new_string =
        "Updated string!";  // The request that will be sent via the service.

    // Wait for services to load.
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Exiting...");
        return 0;
      }
      RCLCPP_INFO(node_->get_logger(), "Service not yet available...");
    }

    // Sending the service request
    auto result = client_->async_send_request(request);

    // Completion of the service request
    if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      std::string temp{result.get()->status.c_str()};
      std::cout << "This happened" << std::endl;
      RCLCPP_INFO(node_->get_logger(), "Response %s", temp.c_str());

      return temp;
    }
    // When the service has failed.
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to call service change_publisher_string");
    return "Service call failed...";
  }
  void TearDown() override { std::cout << "DONE WITH TEARDOWN" << std::endl; }

 protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<cpp_pubsub::srv::ModifyString>::SharedPtr
      client_;  //!< The client that uses the service.
};

TEST_F(TestingFixture, TrueIsTrueTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);
}

TEST_F(TestingFixture, ServiceCallCheck) {
  std::cout << "Service call check" << std::endl;
  EXPECT_EQ(send_service_call(), "STRING CHANGED!");
}
}  // namespace integration_test_alpha

auto main(int argc, char** argv) -> int {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

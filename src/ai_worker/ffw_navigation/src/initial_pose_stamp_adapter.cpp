// Copyright 2026 ROBOTIS CO., LTD.
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
#include <cstdint>
#include <functional>
#include <memory>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ffw_navigation
{

using namespace std::chrono_literals;

class InitialPoseStampAdapter : public rclcpp::Node
{
public:
  InitialPoseStampAdapter()
  : Node("initial_pose_stamp_adapter")
  {
    output_publisher_ = create_publisher<PoseMessage>(
      "/initialpose_nav2", rclcpp::SystemDefaultsQoS());

    readiness_timer_ = create_wall_timer(
      100ms, std::bind(&InitialPoseStampAdapter::check_readiness, this));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for simulation clock and an AMCL subscriber on /initialpose_nav2");
  }

private:
  using PoseMessage = geometry_msgs::msg::PoseWithCovarianceStamped;

  static constexpr int64_t kBackdateNanoseconds = 1'000'000'000LL;

  void check_readiness()
  {
    if (input_subscription_) {
      readiness_timer_->cancel();
      return;
    }

    bool use_sim_time = false;
    if (!get_parameter("use_sim_time", use_sim_time) || !use_sim_time) {
      return;
    }

    const auto now = get_clock()->now();
    if (now.nanoseconds() <= 0 || output_publisher_->get_subscription_count() == 0) {
      return;
    }

    input_subscription_ = create_subscription<PoseMessage>(
      "/initialpose", rclcpp::SystemDefaultsQoS(),
      std::bind(
        &InitialPoseStampAdapter::initial_pose_callback, this,
        std::placeholders::_1));
    readiness_timer_->cancel();

    RCLCPP_INFO(
      get_logger(),
      "Simulation clock and /initialpose_nav2 subscriber are ready; "
      "accepting initial poses on /initialpose");
  }

  void initial_pose_callback(const PoseMessage::SharedPtr message)
  {
    PoseMessage adapted_message = *message;
    const auto now = get_clock()->now();
    const int64_t now_nanoseconds = now.nanoseconds();
    const int64_t adapted_nanoseconds =
      now_nanoseconds > kBackdateNanoseconds ?
      now_nanoseconds - kBackdateNanoseconds : 0;

    adapted_message.header.stamp = static_cast<builtin_interfaces::msg::Time>(
      rclcpp::Time(adapted_nanoseconds, now.get_clock_type()));
    output_publisher_->publish(adapted_message);

    RCLCPP_INFO(
      get_logger(),
      "Forwarded /initialpose to /initialpose_nav2 with simulation stamp "
      "%d.%09u (input stamp %d.%09u)",
      adapted_message.header.stamp.sec,
      adapted_message.header.stamp.nanosec,
      message->header.stamp.sec,
      message->header.stamp.nanosec);
  }

  rclcpp::Publisher<PoseMessage>::SharedPtr output_publisher_;
  rclcpp::Subscription<PoseMessage>::SharedPtr input_subscription_;
  rclcpp::TimerBase::SharedPtr readiness_timer_;
};

}  // namespace ffw_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ffw_navigation::InitialPoseStampAdapter>());
  rclcpp::shutdown();
  return 0;
}

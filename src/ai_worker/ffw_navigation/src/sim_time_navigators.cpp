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

#include <memory>

#include "builtin_interfaces/msg/time.hpp"
#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"
#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ffw_navigation
{

class SimTimeNavigateToPoseNavigator
  : public nav2_bt_navigator::NavigateToPoseNavigator
{
protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override
  {
    auto normalized_goal = std::make_shared<ActionT::Goal>(*goal);
    normalized_goal->pose.header.stamp = builtin_interfaces::msg::Time{};
    return nav2_bt_navigator::NavigateToPoseNavigator::goalReceived(normalized_goal);
  }

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override
  {
    auto normalized_goal = std::make_shared<ActionT::Goal>(*goal);
    normalized_goal->pose.header.stamp = builtin_interfaces::msg::Time{};
    nav2_bt_navigator::NavigateToPoseNavigator::onPreempt(normalized_goal);
  }
};

class SimTimeNavigateThroughPosesNavigator
  : public nav2_bt_navigator::NavigateThroughPosesNavigator
{
protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override
  {
    auto normalized_goal = std::make_shared<ActionT::Goal>(*goal);
    for (auto & pose : normalized_goal->poses) {
      pose.header.stamp = builtin_interfaces::msg::Time{};
    }
    return nav2_bt_navigator::NavigateThroughPosesNavigator::goalReceived(normalized_goal);
  }

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override
  {
    auto normalized_goal = std::make_shared<ActionT::Goal>(*goal);
    for (auto & pose : normalized_goal->poses) {
      pose.header.stamp = builtin_interfaces::msg::Time{};
    }
    nav2_bt_navigator::NavigateThroughPosesNavigator::onPreempt(normalized_goal);
  }
};

}  // namespace ffw_navigation

PLUGINLIB_EXPORT_CLASS(
  ffw_navigation::SimTimeNavigateToPoseNavigator,
  nav2_core::NavigatorBase)

PLUGINLIB_EXPORT_CLASS(
  ffw_navigation::SimTimeNavigateThroughPosesNavigator,
  nav2_core::NavigatorBase)

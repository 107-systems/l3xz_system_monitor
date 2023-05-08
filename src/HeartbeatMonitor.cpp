/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_watchdog/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_watchdog/HeartbeatMonitor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

HeartbeatMonitor::HeartbeatMonitor(
  std::string const & heartbeat_topic,
  std::chrono::milliseconds const heartbeat_timeout,
  rclcpp::Node & node_hdl)
: _heartbeat_timeout{heartbeat_timeout}
, _prev_heartbeat_timepoint{std::chrono::steady_clock::now()}
{
  _heartbeat_sub = node_hdl.create_subscription<std_msgs::msg::UInt64>(
    heartbeat_topic,
    1,
    [this](std_msgs::msg::UInt64::SharedPtr const /* msg */)
    {
      _prev_heartbeat_timepoint = std::chrono::steady_clock::now();
    });

  RCLCPP_INFO(node_hdl.get_logger(), "HeartbeatMonitor subscribed to \"%s\".", heartbeat_topic.c_str());
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool HeartbeatMonitor::isTimeout() const
{
  auto const now = std::chrono::steady_clock::now();
  auto const time_since_last_heartbeat = (now - _prev_heartbeat_timepoint);

  return (std::chrono::duration_cast<std::chrono::milliseconds>(time_since_last_heartbeat) > _heartbeat_timeout);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

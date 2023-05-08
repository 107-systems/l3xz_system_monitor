/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_watchdog/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <chrono>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class HeartbeatMonitor
{
public:
  typedef std::shared_ptr<HeartbeatMonitor> SharedPtr;

  HeartbeatMonitor(
    std::string const & heartbeat_topic,
    std::chrono::milliseconds const heartbeat_timeout,
    rclcpp::Node & node_hdl);


  bool isTimeout() const;


private:
  std::chrono::milliseconds const _heartbeat_timeout;
  std::chrono::steady_clock::time_point _prev_heartbeat_timepoint;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr _heartbeat_sub;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_watchdog/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <map>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>

#include <ros2_heartbeat/monitor/Monitor.h>
#include <ros2_loop_rate_monitor/Monitor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();
  ~Node();

private:
  enum class SystemHealth
  {
    Nominal, Degraded
  };
  SystemHealth _system_health;

  std::map<std::string, heartbeat::Monitor::SharedPtr> _heartbeat_monitor_map;
  enum class NodeLiveliness
  {
    Online, Offline
  };
  std::map<std::string, NodeLiveliness> _heartbeat_liveliness_map;
  void init_heartbeat_monitor();

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _estop_sub;
  bool _is_estop_pressed;
  void init_sub();

  static int8_t constexpr LIGHT_MODE_OFF   = 0;
  static int8_t constexpr LIGHT_MODE_RED   = 1;
  static int8_t constexpr LIGHT_MODE_GREEN = 2;
  static int8_t constexpr LIGHT_MODE_BLUE  = 3;
  static int8_t constexpr LIGHT_MODE_WHITE = 4;
  static int8_t constexpr LIGHT_MODE_AMBER = 5;

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _light_mode_pub;
  void init_pub();

  static std::chrono::milliseconds constexpr WATCHDOG_LOOP_RATE{100};
  loop_rate::Monitor::SharedPtr _watchdog_loop_rate_monitor;
  rclcpp::TimerBase::SharedPtr _watchdog_loop_timer;
  void watchdog_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_watchdog/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_watchdog/Node.h>

#include <sstream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_watchdog")
, _system_health{SystemHealth::Nominal}
, _is_estop_pressed{false}
{
  declare_parameter("heartbeat_monitor_list", std::vector<std::string>{});

  /* Load a list of all nodes to be monitored from
   * the JSON configuration file.
   */
  std::vector<std::string> const heartbeat_monitor_list = get_parameter("heartbeat_monitor_list").as_string_array();

  for (auto &node : heartbeat_monitor_list)
  {
    std::stringstream heartbeat_topic;
    heartbeat_topic << "/l3xz/" << node << "/heartbeat";

    _heartbeat_monitor_map[node] = heartbeat::Monitor::create(
      *this,
      heartbeat_topic.str(),
      [this, node]()
      {
        auto const citer = std::find(_heartbeat_liveliness_lost_list.cbegin(),
                                     _heartbeat_liveliness_lost_list.cend(),
                                     node);
        if (citer != _heartbeat_liveliness_lost_list.cend())
          _heartbeat_liveliness_lost_list.push_back(node);
      },
      [this, node]()
      {
        auto const citer = std::find(_heartbeat_liveliness_lost_list.cbegin(),
                                     _heartbeat_liveliness_lost_list.cend(),
                                     node);
        if (citer != _heartbeat_liveliness_lost_list.cend())
          _heartbeat_liveliness_lost_list.erase(citer);
      },
      [this, node](rclcpp::QOSDeadlineRequestedInfo & event)
      {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5*1000UL,
                             "deadline missed for \"%s\" (total_count: %d, total_count_change: %d).",
                             node.c_str(),
                             event.total_count,
                             event.total_count_change);
      });

    RCLCPP_INFO(get_logger(), "heartbeat monitor active for \"%s\".", node.c_str());
  }

  _estop_sub = create_subscription<std_msgs::msg::Bool>(
    "/l3xz/estop/actual",
    1,
    [this](std_msgs::msg::Bool::SharedPtr const msg)
    {
      _is_estop_pressed = msg->data;
    });

  /* Create publisher object to send the desired
   * light mode to the auxiliary controller of
   * L3X-Z.
   */
  _light_mode_pub = create_publisher<std_msgs::msg::Int8>("/l3xz/light_mode/target", 1);

  /* Setup periodically called function to check the
   * online status (determined via regular received
   * heartbeat messages) of all nodes under monitoring.
   */
  _watchdog_loop_rate_monitor = loop_rate::Monitor::create
    (WATCHDOG_LOOP_RATE, std::chrono::milliseconds(1));
  _watchdog_loop_timer = create_wall_timer
    (std::chrono::milliseconds(WATCHDOG_LOOP_RATE.count()), [this]() { this->watchdog_loop(); });

  RCLCPP_INFO(get_logger(), "Node started successfully.");
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "Node shut down successfully.");
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::watchdog_loop()
{
  _watchdog_loop_rate_monitor->update();
  if (auto const [timeout, opt_timeout_duration] = _watchdog_loop_rate_monitor->isTimeout();
    timeout == loop_rate::Monitor::Timeout::Yes)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "watchdog_loop should be called every %ld ms, but is %ld ms instead",
                         WATCHDOG_LOOP_RATE.count(),
                         opt_timeout_duration.value().count());
  }

  /* Iterate over all registered heartbeat monitors and
   * check if a heartbeat timeout has occurred on any
   * of those.
   */
  std::stringstream heartbeat_no_liveliness_list_ss;
  for (auto const &node : _heartbeat_liveliness_lost_list)
    heartbeat_no_liveliness_list_ss << "\"" << node << "\" ";

  bool const is_heartbeat_timeout = _heartbeat_liveliness_lost_list.size() > 0;
  if (is_heartbeat_timeout)
    RCLCPP_ERROR_THROTTLE(get_logger(),
                          *get_clock(),
                          1000,
                          "heartbeat signal has timed out for nodes: { %s}",
                          heartbeat_no_liveliness_list_ss.str().c_str());

  /* Update system health based on monitoring
   * all relevant nodes.
   */
  if (is_heartbeat_timeout)
    _system_health = SystemHealth::Degraded;
  else
    _system_health = SystemHealth::Nominal;

  /* Set the light mode of L3X-Z dependent
   * on the overall system health and state.
   */
  std_msgs::msg::Int8 light_mode_msg;
  light_mode_msg.data = LIGHT_MODE_WHITE;

  if      (_system_health == SystemHealth::Nominal)
    light_mode_msg.data = LIGHT_MODE_AMBER;

  if (_system_health == SystemHealth::Degraded)
    light_mode_msg.data = LIGHT_MODE_RED;
  else
  {
    if (_is_estop_pressed)
      light_mode_msg.data = LIGHT_MODE_GREEN;
    else
      light_mode_msg.data = LIGHT_MODE_AMBER;
  }

  _light_mode_pub->publish(light_mode_msg);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

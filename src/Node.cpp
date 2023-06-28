/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_system_monitor/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_system_monitor/Node.h>

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
: rclcpp::Node("l3xz_system_monitor")
, _system_health{SystemHealth::Nominal}
, _is_estop_pressed{false}
{
  declare_parameter("heartbeat_monitor_list", std::vector<std::string>{});

  init_heartbeat_monitor();
  init_sub();
  init_pub();

  _watchdog_loop_rate_monitor = loop_rate::Monitor::create(
    WATCHDOG_LOOP_RATE,
    std::chrono::milliseconds(1)
    );
  _watchdog_loop_timer = create_wall_timer(
    WATCHDOG_LOOP_RATE,
    [this]()
    {
      this->watchdog_loop();
    });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "%s shut down.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_heartbeat_monitor()
{
  std::vector<std::string> const heartbeat_monitor_list = get_parameter("heartbeat_monitor_list").as_string_array();

  for (auto const & node : heartbeat_monitor_list)
  {
    /* Count all nodes offline until a liveliness signal
     * has been received.
     */
    _heartbeat_liveliness_map[node] = NodeLiveliness::Offline;

    std::stringstream heartbeat_topic;
    heartbeat_topic << "/l3xz/" << node << "/heartbeat";

    _heartbeat_monitor_map[node] = heartbeat::Monitor::create(
      *this,
      heartbeat_topic.str(),
      [this, node]()
      {
        RCLCPP_ERROR(get_logger(), "liveliness lost for \"%s\".", node.c_str());
        _heartbeat_liveliness_map[node] = NodeLiveliness::Offline;
      },
      [this, node]()
      {
        RCLCPP_INFO(get_logger(), "liveliness gained for \"%s\".", node.c_str());
        _heartbeat_liveliness_map[node] = NodeLiveliness::Online;
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
}

void Node::init_sub()
{
  _estop_sub = create_subscription<std_msgs::msg::Bool>(
    "/l3xz/estop/actual",
    1,
    [this](std_msgs::msg::Bool::SharedPtr const msg)
    {
      _is_estop_pressed = msg->data;
    });
}

void Node::init_pub()
{
  _light_mode_pub = create_publisher<std_msgs::msg::Int8>(
    "/l3xz/light_mode/target",
    1);
}

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
  bool is_heartbeat_timeout = false;
  std::stringstream heartbeat_no_liveliness_list_ss;
  for (auto const & [node, liveliness] : _heartbeat_liveliness_map)
    if (liveliness == NodeLiveliness::Offline)
    {
      is_heartbeat_timeout = true;
      heartbeat_no_liveliness_list_ss << "\n\t" << node;
    }

  if (is_heartbeat_timeout)
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         2*1000UL,
                         "liveliness lost for nodes: %s",
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

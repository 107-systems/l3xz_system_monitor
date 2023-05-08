/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_watchdog/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_watchdog/Node.h>

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
, _prev_watchdog_loop_timepoint{std::chrono::steady_clock::now()}
{
  _watchdog_loop_timer = create_wall_timer
    (std::chrono::milliseconds(WATCHDOG_LOOP_RATE.count()), [this]() { this->watchdog_loop(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::watchdog_loop()
{
  auto const now = std::chrono::steady_clock::now();
  auto const watchdog_loop_rate = (now - _prev_watchdog_loop_timepoint);
  if (watchdog_loop_rate > (WATCHDOG_LOOP_RATE + std::chrono::milliseconds(1)))
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "watchdog_loop should be called every %ld ms, but is %ld ms instead",
                         WATCHDOG_LOOP_RATE.count(),
                         std::chrono::duration_cast<std::chrono::milliseconds>(watchdog_loop_rate).count());
  _prev_watchdog_loop_timepoint = now;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

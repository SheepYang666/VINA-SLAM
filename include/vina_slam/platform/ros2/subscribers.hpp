#pragma once

#include <sensor_msgs/msg/imu.hpp>

void imu_handler(const sensor_msgs::msg::Imu::SharedPtr& msg_in);

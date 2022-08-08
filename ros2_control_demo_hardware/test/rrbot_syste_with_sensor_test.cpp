#include "ros2_control_demo_hardware/rrbot_system_with_sensor.hpp"

#include <gtest/gtest.h>

#include <vector>
#include <thread>
#include <chrono>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

TEST(rrbot_syste_with_sensor_test, get_state_interfaces)
{
  ros2_control_demo_hardware::RRBotSystemWithSensorHardware rrbotSystemWithSensorHardware{};
  std::vector<hardware_interface::StateInterface> stateInterfaces{
    rrbotSystemWithSensorHardware.export_state_interfaces()};
  EXPECT_STREQ(stateInterfaces[0].get_name().data(), "joint1");
  EXPECT_STREQ(stateInterfaces[0].get_interface_name().data(), "position");
  EXPECT_STREQ(stateInterfaces[1].get_name().data(), "joint2");
  EXPECT_STREQ(stateInterfaces[1].get_interface_name().data(), "position");
}

TEST(simulated_servomotor_test, get_command_interfaces)
{
  ros2_control_demo_hardware::RRBotSystemWithSensorHardware rrbotSystemWithSensorHardware{};
  std::vector<hardware_interface::CommandInterface> commandInterfaces{
    rrbotSystemWithSensorHardware.export_command_interfaces()};
  EXPECT_STREQ(commandInterfaces[0].get_name().data(), "joint1");
  EXPECT_STREQ(commandInterfaces[0].get_interface_name().data(), "position");
  EXPECT_STREQ(commandInterfaces[1].get_name().data(), "joint2");
  EXPECT_STREQ(commandInterfaces[1].get_interface_name().data(), "position");
}

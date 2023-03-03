/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"

#include <list>
#include <vector>

#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"

using namespace ez;

// Constructor for integrated encoders
Drive::Drive(double left_width, double right_width, double length, std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int imu_port,
             double wheel_diameter, double ticks, double ratio,
             std::vector<int> middle_tracker_ports,
             double middle_tracker_diameter)
    : imu(imu_port), left_tracker(-1, -1, false), // Default value
      right_tracker(-1, -1, false),               // Default value
      middle_tracker(abs(middle_tracker_ports[0]), abs(middle_tracker_ports[1]),
                    util::is_reversed(middle_tracker_ports[0])),
      left_rotation(-1), right_rotation(-1),
      ez_auto([this] { this->ez_auto_task(); }),
      ez_position_tracker([this] { this->ez_odometry_task(); }),
      l_width(left_width), r_width(right_width),
      length(length) {
  is_tracker = DRIVE_INTEGRATED;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

// Constructor for tracking wheels plugged into the brain
Drive::Drive(double left_width, double right_width, double length, std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int imu_port,
             double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports,
             std::vector<int> right_tracker_ports,
             std::vector<int> middle_tracker_ports,
             double middle_tracker_diameter)
    : imu(imu_port),
      left_tracker(-1, -1,
                   util::is_reversed(left_tracker_ports[0])),
      right_tracker(abs(right_tracker_ports[0]), abs(right_tracker_ports[1]),
                    util::is_reversed(right_tracker_ports[0])),
      middle_tracker(abs(middle_tracker_ports[0]), abs(middle_tracker_ports[1]),
                    util::is_reversed(middle_tracker_ports[0])),
      left_rotation(-1), right_rotation(-1),
      ez_auto([this] { this->ez_auto_task(); }),
      ez_position_tracker([this] { this->ez_odometry_task(); }),
      l_width(left_width), r_width(right_width),
      length(length) {
  is_tracker = DRIVE_ADI_ENCODER;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();

  MIDDLE_TICK_PER_INCH = (ticks * ratio) / (middle_tracker_diameter * M_PI);

  set_defaults();
}

// Constructor for tracking wheels plugged into a 3 wire expander
Drive::Drive(double left_width, double right_width, double length, std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int imu_port,
             double wheel_diameter, double ticks, double ratio,
             std::vector<int> left_tracker_ports,
             std::vector<int> right_tracker_ports, int expander_smart_port,
             std::vector<int> middle_tracker_ports,
             double middle_tracker_diameter)
    : imu(imu_port),
      left_tracker({expander_smart_port, abs(left_tracker_ports[0]),
                    abs(left_tracker_ports[1])},
                   util::is_reversed(left_tracker_ports[0])),
      right_tracker({expander_smart_port, abs(right_tracker_ports[0]),
                     abs(right_tracker_ports[1])},
                    util::is_reversed(right_tracker_ports[0])),
      middle_tracker(abs(middle_tracker_ports[0]), abs(middle_tracker_ports[1]),
                    util::is_reversed(middle_tracker_ports[0])),
      left_rotation(-1), right_rotation(-1),
      ez_auto([this] { this->ez_auto_task(); }),
      ez_position_tracker([this] { this->ez_odometry_task(); }),
      l_width(left_width), r_width(right_width),
      length(length) {
  is_tracker = DRIVE_ADI_ENCODER;

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = ticks;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

// Constructor for rotation sensors
Drive::Drive(double left_width, double right_width, double length, std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int imu_port,
             double wheel_diameter, double ratio, int left_rotation_port,
             int right_rotation_port,
             std::vector<int> middle_tracker_ports,
             double middle_tracker_diameter)
    : imu(imu_port), left_tracker(-1, -1, false), // Default value
      right_tracker(-1, -1, false),               // Default value
      middle_tracker(abs(middle_tracker_ports[0]), abs(middle_tracker_ports[1]),
                    util::is_reversed(middle_tracker_ports[0])),
      left_rotation(abs(left_rotation_port)),
      right_rotation(abs(right_rotation_port)),
      ez_auto([this] { this->ez_auto_task(); }),
      ez_position_tracker([this] { this->ez_odometry_task(); }),
      l_width(left_width), r_width(right_width),
      length(length) {
  is_tracker = DRIVE_ROTATION;
  left_rotation.set_reversed(util::is_reversed(left_rotation_port));
  right_rotation.set_reversed(util::is_reversed(right_rotation_port));

  // Set ports to a global vector
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }

  // Set constants for tick_per_inch calculation
  WHEEL_DIAMETER = wheel_diameter;
  RATIO = ratio;
  CARTRIDGE = 4096;
  TICK_PER_INCH = get_tick_per_inch();

  set_defaults();
}

void Drive::initialize() {
  init_curve_sd();
  imu_calibrate();
  reset_drive_sensor();
  reset_position();
}
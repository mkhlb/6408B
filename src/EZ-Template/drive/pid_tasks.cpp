/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.hpp"

using namespace ez;

void Drive::ez_auto_task() {
  while (true) {
    // Autonomous PID
    if (get_mode() == DRIVE)
      drive_pid_task();
    else if (get_mode() == TURN)
      turn_pid_task();
    else if (get_mode() == SWING)
      swing_pid_task();
    else if (get_mode() == POINT)
      point_pid_task();
    else if (get_mode() == PATH)
      path_pid_task();

    if (pros::competition::is_autonomous() && !util::AUTON_RAN)
      util::AUTON_RAN = true;
    //else if (!pros::competition::is_autonomous())
      //set_mode(DISABLE);

    pros::delay(util::DELAY_TIME);
  }
}

// Drive PID task
void Drive::drive_pid_task() {
  // Compute PID
  leftPID.compute(left_sensor());
  rightPID.compute(right_sensor());
  headingPID.compute(get_gyro());

  // Compute slew
  double l_slew_out = slew_calculate(left_slew, left_sensor());
  double r_slew_out = slew_calculate(right_slew, right_sensor());

  // Clip leftPID and rightPID to slew (if slew is disabled, it returns max_speed)
  double l_drive_out = util::clip_num(leftPID.output, l_slew_out, -l_slew_out);
  double r_drive_out = util::clip_num(rightPID.output, r_slew_out, -r_slew_out);

  // Toggle heading
  double gyro_out = heading_on ? headingPID.output : 0;

  // Combine heading and drive
  double l_out = l_drive_out + gyro_out;
  double r_out = r_drive_out - gyro_out;

  // Set motors
  if (drive_toggle)
    set_tank(l_out, r_out);
}

// Turn PID task
void Drive::turn_pid_task() {
  // Compute PID
  turnPID.compute(get_gyro());

  // Clip gyroPID to max speed
  double gyro_out = util::clip_num(turnPID.output, max_speed, -max_speed);

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (turnPID.constants.ki != 0 && (fabs(turnPID.get_target()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
    if (get_turn_min() != 0)
      gyro_out = util::clip_num(gyro_out, get_turn_min(), -get_turn_min());
  }

  // Set motors
  if (drive_toggle)
    set_tank(gyro_out, -gyro_out);
}

void Drive::point_pid_task() {
  if((point_target - position).get_magnitude() > 6) {
    //figure out offset
    Angle offset = Angle();
    switch (point_orientation) {
      case FORWARD: offset = Angle::from_deg(0); break;
      case BACKWARD: offset = Angle::from_deg(180); break;
      case AGNOSTIC: {
        if(abs(error_to_point(point_target)) > 180) {
          offset = Angle::from_deg(180);
        }
        else {
          offset = Angle::from_deg(0);
        }
        break;
      }
    }
    //set PIDs
    set_point_heading_pid(point_target);
    set_straight_point_drive_pid(point_target, max_speed, false, true, false);
    
  }
  else if((point_target - position).get_magnitude() > 4) {
    set_straight_point_drive_pid(point_target, max_speed, false, true, false);
  }
  else {
    set_straight_point_drive_pid(point_target, max_speed);
  }
}

void Drive::path_pid_task() {

}

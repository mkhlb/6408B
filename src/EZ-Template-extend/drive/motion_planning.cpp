#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "pros/rtos.hpp"

void Drive::set_target_relative_turn_pid(double target, int speed) {
  // Compute absolute target by adding to current heading
  double absolute_target = headingPID.get_target() + target;

  set_turn_pid(absolute_target, speed);
}

void Drive::set_heading_relative_turn_pid(double target, int speed) {
  // Compute absolute target by adding to current heading
  double absolute_target = get_gyro() + target;

  set_turn_pid(absolute_target, speed);
}

void Drive::set_point_turn_pid(Vector2 target, int speed, Angle offset) {
  //calculate direction vector from current position to target
  Vector2 position_to_target_unit = (target - position).get_normalized();

  double angle = Angle::shortest_error(orientation, position_to_target_unit.get_angle_direction() + offset);
  //set_turn_pid(position_to_target_unit.get_angle_direction().get_deg() + offset.get_deg(), 100);
  set_heading_relative_turn_pid(angle * Angle::RAD_TO_DEG, speed);
}

double Drive::error_to_point(Vector2 target, Angle offset) {
  //calculate direction vector from current position to target
  Vector2 position_to_target_unit = (target - position).get_normalized();

  double angle = Angle::shortest_error(orientation, position_to_target_unit.get_angle_direction() + offset);
  //set_turn_pid(position_to_target_unit.get_angle_direction().get_deg() + offset.get_deg(), 100);
  return angle * Angle::RAD_TO_DEG;
}

void Drive::set_straight_point_drive_pid(Vector2 target, int speed) {
  Vector2 position_to_target = target - position;
  Vector2 orientation_unit = Vector2::from_polar(1, orientation);
  
  double projected = position_to_target * orientation_unit; // dot product with a unit vector is just a regular projection
  set_drive_pid(projected, speed);
}

void Drive::set_orientation_turn_pid(Angle target, int speed) {
  double error = Angle::shortest_error(orientation, target);

  set_heading_relative_turn_pid(error * Angle::RAD_TO_DEG, speed);
}

void Drive::set_heading_relative_swing_pid(e_swing type, double target, int speed, double offside_multiplier) {
  set_swing_pid(type, get_gyro() + target, speed, offside_multiplier);
}

void Drive::set_target_relative_swing_pid(e_swing type, double target, int speed, double offside_multiplier) {
  set_swing_pid(type, headingPID.get_target() + target, speed, offside_multiplier);
}

void Drive::set_orientation_swing_pid(e_swing swing_type, Angle target, int speed, double offside_multiplier) {
  set_heading_relative_swing_pid(swing_type, Angle::shortest_error(orientation, target) * Angle::RAD_TO_DEG, speed, offside_multiplier);
}

void Drive::wait_until_heading_relative(double target) {
  wait_until(get_gyro() + target);
}

void Drive::wait_until_target_relative(double target) {
  if(mode == TURN) {
    wait_until(turnPID.get_target() + target);
  }
  else if(mode == SWING) {
    wait_until(swingPID.get_target() + target);
  }
}

void Drive::wait_until_orientation(Angle target) {
  wait_until_heading_relative(Angle::shortest_error(orientation, target) * Angle::RAD_TO_DEG);
}

void Drive::drive_to_point(Vector2 target, int speed, bool backwards) {
  while(headingPID.exit_condition() == ez::RUNNING) {
    Angle offset = Angle();
    if(backwards) { offset = Angle::from_deg(180); }
    set_point_turn_pid(target, speed, offset);
    set_straight_point_drive_pid(target, speed);
    pros::delay(util::DELAY_TIME);
  }
  set_straight_point_drive_pid(target, speed);
  wait_drive();
}
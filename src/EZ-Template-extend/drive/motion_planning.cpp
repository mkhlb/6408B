#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/drive/drive.hpp"

void Drive::set_relative_turn_pid(double target, int speed) {
  // Compute absolute target by adding to current heading
  double absolute_target = turnPID.get_target() + target;

  // Print targets
  if (print_toggle) printf("Turn Started... Target Value: %f\n", absolute_target);

  // Set PID targets
  turnPID.set_target(absolute_target);
  headingPID.set_target(absolute_target);
  set_max_speed(speed);

  // Run task
  set_mode(TURN);
}

void Drive::set_point_turn_pid(Vector2 target, int speed, Angle offset) {
  //calculate direction vector from current position to target
  Vector2 position_to_target_unit = (target - position).get_normalized();

  double angle = Angle::shortest_error(orientation, -position_to_target_unit.get_angle_direction());
  //set_turn_pid(position_to_target_unit.get_angle_direction().get_deg(), 100);
  set_relative_turn_pid(angle * Angle::RAD_TO_DEG, speed);
}

void Drive::set_straight_point_drive_pid(Vector2 target, int speed) {
  Vector2 position_to_target_unit = (target - position).get_normalized();
  Vector2 orientation_unit = Vector2::from_polar(1, orientation);
  
  double projected = position_to_target_unit * orientation_unit; // dot product with a unit vector is just a regular projection

  set_drive_pid(projected, speed);
}

void Drive::set_orientation_turn_pid(Angle target, int speed) {
  double error = Angle::shortest_error(orientation, target);

  set_relative_turn_pid(error * Angle::RAD_TO_DEG, speed);
}
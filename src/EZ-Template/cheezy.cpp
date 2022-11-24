#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.h"
#include <algorithm>
#include <cmath>

void Drive::arcade_mkhl_standard(e_type stick_type, double interpolator_start, double interpolator_end) {
  is_tank = false;
  reset_drive_sensors_opcontrol();

  // Toggle for controller curve
  modify_curve_with_controller();

  int fwd_stick, turn_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (stick_type == SPLIT) {
    // Put the joysticks through the curve function
    
    fwd_stick = left_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    turn_stick = right_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
  } else if (stick_type == SINGLE) {
    // Put the joysticks through the curve function
    fwd_stick = left_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    turn_stick = right_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
  }

  double interpolator = (fabs(fwd_stick)-(interpolator_start))/(interpolator_end - interpolator_start);

  mkhl(fwd_stick, turn_stick, interpolator);
}

void Drive::arcade_mkhl_flipped(e_type stick_type, double interpolator_start, double interpolator_end) {
  is_tank = false;
  reset_drive_sensors_opcontrol();

  // Toggle for controller curve
  modify_curve_with_controller();

  int turn_stick, fwd_stick;
  // Check arcade type (split vs single, normal vs flipped)
  if (stick_type == SPLIT) {
    // Put the joysticks through the curve function
    fwd_stick = right_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    turn_stick = left_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
  } else if (stick_type == SINGLE) {
    // Put the joysticks through the curve function
    fwd_stick = right_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    turn_stick = left_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
  }

  double interpolator = (fabs(fwd_stick)-(interpolator_start))/(interpolator_end - interpolator_start);

  mkhl(fwd_stick, fwd_stick, interpolator);
}

void Drive::mkhl(double forward_stick, double turn_stick, double interpolator) {
  //scale between -1.0 and 1.0
  forward_stick = forward_stick / 127;
  turn_stick = turn_stick / 127;

  //Curvature
  double left_curvature = forward_stick + fabs(forward_stick) * turn_stick;
  double right_curvature = forward_stick - fabs(forward_stick) * turn_stick;
  double max_curvature = max(fabs(left_curvature), fabs(right_curvature));

  //normalize output
  if (max_curvature > 1.0) {
    left_curvature /= max_curvature;
    right_curvature /= max_curvature;
  }

  // Find normal tank output (for low speeds/ point turns)
  double left_tank = util::clip_num(forward_stick + turn_stick, 1.0, -1.0);
  double right_tank = util::clip_num(forward_stick - turn_stick, 1.0, -1.0);

  //interpolate so that at high speeds (high interpolator values) rely more on cheezy and low speeds rely more on tank
  double left_speed = left_curvature * interpolator + left_tank*(1-interpolator);
  double right_speed = right_curvature * interpolator + right_tank*(1-interpolator);

  // set to drive motors
  joy_thresh_opcontrol(left_speed * 127, right_speed * 127);
}

void Drive::set_drive_slew(double accel, double decel) {
  acceleration = accel;
  deceleration = decel;
}

void Drive::joy_thresh_opcontrol(int l_stick, int r_stick) {
  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick) > JOYSTICK_THRESHOLD || abs(r_stick) > JOYSTICK_THRESHOLD) {
    //slew!
    double left_delta = l_stick - last_left_speed;
    double right_delta = r_stick - last_right_speed;

    // acceleration in percents per second, we need it per ticks

    //clamp delta
    left_delta = util::sgn(left_delta) * util::clip_num(fabs(left_delta), acceleration * ez::util::DELAY_TIME / 1000, deceleration * ez::util::DELAY_TIME / 1000);
    right_delta = util::sgn(right_delta) * util::clip_num(fabs(right_delta), acceleration * ez::util::DELAY_TIME / 1000, deceleration * ez::util::DELAY_TIME / 1000);

    double left_speed = l_stick + left_delta;
    double right_speed = r_stick + right_delta;

    set_tank(l_stick, r_stick);

    last_left_speed = left_speed;
    last_right_speed = right_speed;

    if (active_brake_kp != 0) reset_drive_sensor();
  }
  // When joys are released, run active brake (P) on drive
  else {
    set_tank((0 - left_sensor()) * active_brake_kp, (0 - right_sensor()) * active_brake_kp);
  }
}

void Drive::set_defaults() {
  // PID Constants
  headingPID = {11, 0, 20, 0};
  forward_drivePID = {0.45, 0, 5, 0};
  backward_drivePID = {0.45, 0, 5, 0};
  turnPID = {5, 0.003, 35, 15};
  swingPID = {7, 0, 45, 0};
  leftPID = {0.45, 0, 5, 0};
  rightPID = {0.45, 0, 5, 0};
  set_turn_min(30);
  set_swing_min(30);

  // Slew constants
  set_slew_min_power(80, 80);
  set_slew_distance(7, 7);

  // Exit condition constants
  set_exit_condition(turn_exit, 100, 3, 500, 7, 500, 500);
  set_exit_condition(swing_exit, 100, 3, 500, 7, 500, 500);
  set_exit_condition(drive_exit, 80, 50, 300, 150, 500, 500);

  // Modify joystick curve on controller (defaults to disabled)
  toggle_modify_curve_with_controller(true);

  // Left / Right modify buttons
  set_left_curve_buttons(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
  set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Enable auto printing and drive motors moving
  toggle_auto_drive(true);
  toggle_auto_print(true);

  // Disables limit switch for auto selector
  as::limit_switch_lcd_initialize(nullptr, nullptr);
}
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

  double interpolator = (fabs(fwd_stick)-interpolator_start)/(interpolator_end - interpolator_start);

  mkhl(fwd_stick, turn_stick, ez::util::clip_num(interpolator, 1, 0));
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

  mkhl(fwd_stick, fwd_stick, ez::util::clip_num(interpolator, 1, 0));
}

void Drive::mkhl(int forward_stick, int turn_stick, double interpolator) {
  //scale between -1.0 and 1.0
  double forward_stick_clamped = (double)forward_stick / 127.0;
  double turn_stick_clamped = (double)turn_stick / 127.0;

  //Curvature
  double left_curvature = forward_stick_clamped + fabs(forward_stick_clamped) * turn_stick_clamped;
  double right_curvature = forward_stick_clamped - fabs(forward_stick_clamped) * turn_stick_clamped;
  double max_curvature = max(fabs(left_curvature), fabs(right_curvature));

  //normalize output
  if (max_curvature > 1.0) {
    left_curvature /= max_curvature;
    right_curvature /= max_curvature;
  }

  // Find normal tank output (for low speeds/ point turns)
  double left_tank = util::clip_num(forward_stick_clamped + turn_stick_clamped, 1.0, -1.0);
  double right_tank = util::clip_num(forward_stick_clamped - turn_stick_clamped, 1.0, -1.0);

  //interpolate so that at high speeds (high interpolator values) rely more on cheezy and low speeds rely more on tank
  double left_speed = left_curvature * interpolator + left_tank*(1.0-interpolator);
  double right_speed = right_curvature * interpolator + right_tank*(1.0-interpolator);

  // set to drive motors
  joy_thresh_opcontrol(left_speed * 127.0, right_speed * 127.0);
}

void Drive::arcade_normalized_standard(e_type stick_type) {
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

  int left_power = fwd_stick + turn_stick;
  int right_power = fwd_stick - turn_stick;
  int max_power = max(fabs(fwd_stick), fabs(turn_stick));

  if(max_power > 127){
    left_power /= max_power;
    right_power /= max_power;
  }

  joy_thresh_opcontrol(left_power, right_power);
}

void Drive::arcade_normalized_flipped(e_type stick_type) {
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

  int left_power = fwd_stick + turn_stick;
  int right_power = fwd_stick - turn_stick;
  int max_power = max(fabs(fwd_stick), fabs(turn_stick));

  if(max_power > 127){
    left_power /= max_power;
    right_power /= max_power;
  }

  joy_thresh_opcontrol(left_power, right_power);
}



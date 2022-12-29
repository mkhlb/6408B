#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "pros/rtos.hpp"

void Drive::set_target_relative_turn_pid(double target, int speed, bool mode_set) {
  // Compute absolute target by adding to current heading
  double absolute_target = headingPID.get_target() + target;

  set_turn_pid(absolute_target, speed, mode_set);
}

void Drive::set_heading_relative_turn_pid(double target, int speed, bool mode_set) {
  // Compute absolute target by adding to current heading
  double absolute_target = get_gyro() + target;

  set_turn_pid(absolute_target, speed, mode_set);
}

void Drive::set_point_turn_pid(Vector2 target, int speed, Angle offset, bool mode_set) {
  set_heading_relative_turn_pid(error_to_point(target, offset), speed, mode_set);
}

double Drive::error_to_point(Vector2 target, Angle offset) {
  //calculate direction vector from current position to target
  Vector2 position_to_target_unit = (target - position).get_normalized();

  double angle = Angle::shortest_error(orientation, position_to_target_unit.get_angle_direction() + offset);
  //set_turn_pid(position_to_target_unit.get_angle_direction().get_deg() + offset.get_deg(), 100);
  return angle * Angle::RAD_TO_DEG;
}

double Drive::straight_to_point(Vector2 target) {
  Vector2 position_to_target = target - position;
  Vector2 orientation_unit = Vector2::from_polar(1, orientation);
  
  return position_to_target * orientation_unit; // dot product with a unit vector is just a regular projection
}

void Drive::set_straight_point_drive_pid(Vector2 target, int speed, bool slew_on, bool heading, bool mode_set) {
  
  set_drive_pid(straight_to_point(target), speed, slew_on, heading, mode_set);
}

void Drive::set_orientation_turn_pid(Angle target, int speed, bool mode_set) {
  double error = Angle::shortest_error(orientation, target);

  set_heading_relative_turn_pid(error * Angle::RAD_TO_DEG, speed, mode_set);
}

void Drive::set_heading_relative_swing_pid(e_swing type, double target, int speed, double offside_multiplier, bool mode_set) {
  set_swing_pid(type, get_gyro() + target, speed, offside_multiplier, mode_set);
}

void Drive::set_target_relative_swing_pid(e_swing type, double target, int speed, double offside_multiplier, bool mode_set) {
  set_swing_pid(type, headingPID.get_target() + target, speed, offside_multiplier, mode_set);
}

void Drive::set_orientation_swing_pid(e_swing swing_type, Angle target, int speed, double offside_multiplier, bool mode_set) {
  set_heading_relative_swing_pid(swing_type, Angle::shortest_error(orientation, target) * Angle::RAD_TO_DEG, speed, offside_multiplier, mode_set);
}

void Drive::wait_until_heading(double target) {
  if (mode == TURN || mode == SWING) {
    // Calculate error between current and target (target needs to be an in between position)
    int g_error = target - get_gyro();
    int g_sgn = util::sgn(g_error);

    exit_output turn_exit = RUNNING;
    exit_output swing_exit = RUNNING;

    pros::Motor& sensor = current_swing == ez::LEFT_SWING ? left_motors[0] : right_motors[0];

    while (true) {
      g_error = target - get_gyro();

      // If turning...
      if (mode == TURN) {
        // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
        if (util::sgn(g_error) == g_sgn) {
          if (turn_exit == RUNNING) {
            turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
            pros::delay(util::DELAY_TIME);
          } else {
            if (print_toggle) std::cout << "  Turn: " << exit_to_string(turn_exit) << " Wait Until Exit.\n";

            if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
              interfered = true;
            }
            return;
          }
        }
        // Once we've past target, return
        else if (util::sgn(g_error) != g_sgn) {
          if (print_toggle) std::cout << "  Turn Wait Until Exit.\n";
          return;
        }
      }

      // If swinging...
      else {
        // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
        if (util::sgn(g_error) == g_sgn) {
          if (swing_exit == RUNNING) {
            swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition(sensor);
            pros::delay(util::DELAY_TIME);
          } else {
            if (print_toggle) std::cout << "  Swing: " << exit_to_string(swing_exit) << " Wait Until Exit.\n";

            if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
              interfered = true;
            }
            return;
          }
        }
        // Once we've past target, return
        else if (util::sgn(g_error) != g_sgn) {
          if (print_toggle) std::cout << "  Swing Wait Until Exit.\n";
          return;
        }
      }

      pros::delay(util::DELAY_TIME);
    }
  }
}

void Drive::wait_until_distance_travelled(double target) {
  if (mode == DRIVE) {
    // Calculate error between current and target (target needs to be an in between position)
    int l_tar = l_start + (target * TICK_PER_INCH);
    int r_tar = r_start + (target * TICK_PER_INCH);
    int l_error = l_tar - left_sensor();
    int r_error = r_tar - right_sensor();
    int l_sgn = util::sgn(l_error);
    int r_sgn = util::sgn(r_error);

    exit_output left_exit = RUNNING;
    exit_output right_exit = RUNNING;

    while (true) {
      l_error = l_tar - left_sensor();
      r_error = r_tar - right_sensor();

      // Before robot has reached target, use the exit conditions to avoid getting stuck in this while loop
      if (util::sgn(l_error) == l_sgn || util::sgn(r_error) == r_sgn) {
        if (left_exit == RUNNING || right_exit == RUNNING) {
          left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors[0]);
          right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors[0]);
          pros::delay(util::DELAY_TIME);
        } else {
          if (print_toggle) std::cout << "  Left: " << exit_to_string(left_exit) << " Wait Until Exit.   Right: " << exit_to_string(right_exit) << " Wait Until Exit.\n";

          if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
            interfered = true;
          }
          return;
        }
      }
      // Once we've past target, return
      else if (util::sgn(l_error) != l_sgn || util::sgn(r_error) != r_sgn) {
        if (print_toggle) std::cout << "  Drive Wait Until Exit.\n";
        return;
      }

      pros::delay(util::DELAY_TIME);
    }
  }
  else if(mode == POINT || mode == PATH) {
    while(true) {
      if((position - point_start).get_magnitude() > target) {
        return;
      }
      if(mode == DRIVE && leftPID.exit_condition() != RUNNING && rightPID.exit_condition() != RUNNING) { // if transitioned to straight drive and exited leave!
        return;
      }
      pros::delay(util::DELAY_TIME);
    }
  }
}

void Drive::wait_until_distance_remaining(double target) {
  if (mode == DRIVE) {
    double travelled_target = target * TICK_PER_INCH;
    wait_until_distance_travelled((leftPID.get_target() - l_start) - travelled_target);
  }
  else if(mode == POINT || mode == PATH) {
    while(true) {
      if(mode == POINT && (point_target - position).get_magnitude() < target) {
        return;
      }
      if(mode == PATH && (*path.end() - position).get_magnitude() < target) {
        return;
      }
      if(mode == DRIVE && leftPID.exit_condition() != RUNNING && rightPID.exit_condition() != RUNNING) { // if transitioned to straight drive and exited leave!
        return;
      }
      pros::delay(util::DELAY_TIME);
    }
  }
}

void Drive::wait_until_heading_relative(double target) {
  wait_until_heading(get_gyro() + target);
}

void Drive::wait_until_target_relative(double target) {
  if(mode == TURN) {
    wait_until_heading(turnPID.get_target() + target);
  }
  else if(mode == SWING) {
    wait_until_heading(swingPID.get_target() + target);
  }
}

void Drive::wait_until_orientation(Angle target) {
  wait_until_heading_relative(Angle::shortest_error(orientation, target) * Angle::RAD_TO_DEG);
}

void Drive::set_orientation_heading_pid(Angle target) {
  headingPID.set_target(Angle::shortest_error(orientation, target) * Angle::RAD_TO_DEG);
}

void Drive::set_target_relative_heading_pid(double target) {
  headingPID.set_target(headingPID.get_target() + target);
}

void Drive::set_point_heading_pid(Vector2 target, Angle offset) {
  set_heading_relative_heading_pid(error_to_point(target, offset));
}

void Drive::set_heading_relative_heading_pid(double target) {
  headingPID.set_target(get_gyro() + target);
}

void Drive::set_point_drive_pid(Vector2 target, int speed, e_point_orientation orientation) {
  point_target = target;
  point_start = position;
  set_max_speed(speed);
  point_orientation = orientation;
  headingPID.reset_variables();
  set_mode(POINT);
}

void Drive::set_point_path_orientation(e_point_orientation orientation) {
  point_orientation = orientation;
}
#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

using namespace ez;

void Drive::ez_odometry_task() {
  int last_left_sensor = left_sensor();
  int last_right_sensor = right_sensor();

  Angle last_orientation = orientation;

  while (true) {
    Vector2 local_move(0, 0);

    double left_distance = (double)left_sensor() / TICK_PER_INCH;
    double right_distance = (double)right_sensor() / TICK_PER_INCH;

    last_left_sensor = left_sensor();
    last_right_sensor = right_sensor();

    double orientation_delta = orientation.get_rad() - last_orientation.get_rad();

    double orientation_average =
        last_orientation.get_rad() + orientation_delta / 2;

    if (orientation_delta == 0) {
      local_move = Vector2((left_distance + right_distance) / 2, 0);
    } else {
      local_move =
          Vector2(((left_distance / orientation_delta + width / 2) +
                   (right_distance / orientation_delta - width)) /
                      2,
                  0);
    }

    if (local_move.get_magnitude() != 0) {
      Vector2 global_move = Vector2(local_move.x, local_move.y);

      global_move.set_angle_direction(local_move.get_angle_direction().get_rad() +
                                    orientation_average);

      position.x += global_move.x;
      position.y += global_move.y;
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

void Drive::reset_position(Vector2 position, Angle w) {
  position = position;
  orientation = w;
}
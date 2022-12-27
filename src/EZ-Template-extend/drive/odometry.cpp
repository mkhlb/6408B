#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

using namespace ez;

void Drive::ez_odometry_task() { //COORDINATE SYSTEM: at orientation 0 robot moves towards positive x, clockwise turn is positive (CW+)
  int last_left_sensor = left_sensor();
  int last_right_sensor = right_sensor();
  int last_middle_sensor = middle_sensor();

  Angle last_orientation = Angle::from_rad(orientation.get_rad());

  printf("starting odom task");

  while (true) {

    if(imu.is_calibrating()) {
      continue;
    }

    Vector2 local_move(0, 0);

    orientation.set_deg(get_gyro());

    double left_distance = (double)(left_sensor() - last_left_sensor) / TICK_PER_INCH;
    double right_distance = (double)(right_sensor() - last_right_sensor) / TICK_PER_INCH;
    double middle_distance = (double)(middle_sensor() - last_middle_sensor) / MIDDLE_TICK_PER_INCH;

    last_left_sensor = left_sensor();
    last_right_sensor = right_sensor();
    last_middle_sensor = middle_sensor();

    double orientation_delta = Angle::shortest_error(last_orientation, orientation);

    Angle orientation_average = Angle::from_rad(last_orientation.get_rad() + orientation_delta / 2);

    //printf("Average Orientation: %f", (float)orientation_average.get_deg());

    last_orientation.set_rad(orientation.get_rad());

    if (orientation_delta == 0) {
      local_move = Vector2((left_distance + right_distance) / 2, 0);
    } else {
      // arc length = radius * theta
      // find radius from both sides and add or subtract the offset from tracking center (by convention the arc is to the right of robot so L is far from arc center and R is close)

      double left_radius = left_distance / orientation_delta;
      double right_radius = right_distance / orientation_delta;
      double middle_radius = ((left_radius - width / 2) + (right_radius + width / 2)) / 2; // take average of both side's reading of radius 
      double lateral_radius = (middle_distance / orientation_delta) + length;
      local_move = // local move = chord length = 2 * radius * sin(theta/2)
          Vector2(2 * middle_radius * sin(orientation_delta / 2) , 2 * lateral_radius * sin(orientation_delta / 2));
    }
    if (local_move.get_magnitude() != 0) {
      Vector2 global_move = Vector2(local_move.x, local_move.y);

      // local move is offset from global CS by average_orientation so we must rotate it
      global_move.set_angle_direction( -local_move.get_angle_direction() + orientation_average); // local move.get_angle_direction is in coordinate system where CCW+ so we must invert it
      

      position.x += global_move.x;
      position.y += global_move.y; 
      //printf("Global move x: %f, y: %f", global_move.x, global_move.y);
    }

    pros::delay(ez::util::DELAY_TIME);
    
  }
}

void Drive::reset_position(Vector2 set_position, Angle w) {
  position = set_position;
  set_angle(w.get_deg());
  orientation = w;
  
}
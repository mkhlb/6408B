#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "pros/rtos.hpp"
#include <iterator>
#include <sys/types.h>
#include <vector>

void Drive::reset_path() {
  path.clear();
}

void Drive::add_point(Vector2 point) {
  path.push_back(point);
}

void Drive::add_points(std::list<Vector2> points) {
  for(auto it = points.cbegin(); it != points.cend(); ++it) {
    path.push_back(*it);
  }
}

void Drive::drive_to_points(int speed) {
  for(auto it = path.begin(); it != path.end(); ++it) {
    set_point_drive_pid(*it, speed);
  }
}

void Drive::path_set_target() {

  std::vector<Vector2>::iterator it = path.begin();
  if(path_advance > 0) { std::advance(it, path_advance); }

  Vector2 first_point = *it;
  ++it;
  Vector2 second_point = *it;

  Vector2 local_first_point = first_point - position;
  Vector2 local_second_point = second_point - position;

  if(local_second_point.get_magnitude() < path_lookahead) { // second point is within lookahead, go right to it and start searching next line segment
    point_target = second_point;
    path_advance++; // if point is found!
  }
  else {
    double line_x = local_second_point.x - local_first_point.x;
    double line_y = local_second_point.y - local_first_point.y;
    double line_length = sqrt(line_x*line_x + line_y*line_y);
    double cross = local_first_point.x * local_second_point.y - local_second_point.x * local_first_point.y;
    double discriminant = path_lookahead*path_lookahead*line_length*line_length - cross*cross;

    if(discriminant >= 0) { // found!
      Vector2 first_intersect = Vector2(
        (cross * line_y + util::sgn(line_y) * line_x * sqrt(discriminant)) / (line_length * line_length),
        (-cross * line_x + fabs(line_y) * sqrt(discriminant)) / (line_length * line_length)
      );
      Vector2 second_intersect = Vector2(
        (cross * line_y - util::sgn(line_y) * line_x * sqrt(discriminant)) / (line_length * line_length),
        (-cross * line_x - fabs(line_y) * sqrt(discriminant)) / (line_length * line_length)
      );
      bool first_intersect_legal = false;
      bool second_intersect_legal = false;
      
      Vector2 lower_bound = Vector2(fmin(local_first_point.x, local_second_point.x), fmin(local_first_point.y, local_second_point.y));
      Vector2 upper_bound = Vector2(fmax(local_first_point.x, local_second_point.x), fmax(local_first_point.y, local_second_point.y));

      if(first_intersect.x > lower_bound.x && first_intersect.y > lower_bound.y && first_intersect.x < upper_bound.x && first_intersect.y < upper_bound.y) {
        first_intersect_legal = true;
      }
      if(second_intersect.x > lower_bound.x && second_intersect.y > lower_bound.y && second_intersect.x < upper_bound.x && second_intersect.y < upper_bound.y) {
        second_intersect_legal = true;
      }

      if(first_intersect_legal && second_intersect_legal) {
        if((first_intersect - local_second_point).get_magnitude() < (second_intersect - local_second_point).get_magnitude()) { // find if first point is closer to second path point
          point_target = first_intersect + position;
        }
        else {
          point_target = second_intersect + position;
        }
      }
      else if(first_intersect_legal) {
        point_target = first_intersect + position;
      }
      else if(second_intersect_legal) {
        point_target = second_intersect + position;
      }

      
    }
    else { // nothing :[
    }
  }
}

void Drive::set_path_pid(int speed, double lookahead, e_point_orientation orientation) {
  path_lookahead = lookahead;
  set_max_speed(speed);
  point_orientation = orientation;
  path_advance = 0;
  point_target = path[0];

  set_mode(PATH);
}

void Drive::wait_until_points_passed(int target) {
  int relative_target = target + path_advance;
  while(true) {
    if(path_advance >= relative_target) {
      return;
    }
    if(mode != PATH) {
      wait_drive();
      return;
    }
  }
}
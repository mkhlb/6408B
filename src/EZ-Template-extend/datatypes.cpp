#include "EZ-Template/datatypes.hpp"

#include <math.h>

using namespace ez;

double operator*(Vector2 const &obj1, Vector2 const &obj2) {return obj1.x * obj2.x + obj1.y * obj2.y;}

Vector2 operator+(Vector2 const &obj1, Vector2 const &obj2) {
    return Vector2(obj1.x + obj2.x, obj1.y + obj2.y);
  }

Vector2 operator*(double const &scal, Vector2 const &vec) { return Vector2(vec.x * scal, vec.y * scal); }

Vector2 operator*(Vector2 const &vec, double const &scal) {return scal * vec;};

double Vector2::get_magnitude() { return sqrt(x * x + y * y); }
Angle Vector2::get_angle_direction() {
  if(x == 0) {
    if(y >= 0) {
      return Angle::from_deg(90);
    }
    else {
      return Angle::from_deg(-90);
    }
  }

  return Angle::from_rad(atan2(y, x));
}
Vector2 Vector2::get_normalized() {
  if(get_magnitude() == 0) {
    return Vector2(0,0);
  }
  return Vector2(x / get_magnitude(), y / get_magnitude());
}
void Vector2::set_magnitude(double value) {
  double mag = get_magnitude();

  x = x / mag * value;

  y = y / mag * value;
}
void Vector2::set_angle_direction(Angle value) {
  double mag = get_magnitude();

  x = mag * cos(value.get_rad());

  y = mag * sin(value.get_rad());
}

double Angle::shortest_error(Angle from,
                            Angle to) // returns shortest error from angle this
                                      // method is being called on to other
{
  Angle error = to-from;
  // must clamp between pi and -pi
  
  if(error.get_deg() > 180) {
    return error.get_rad() - 360 * DEG_TO_RAD;
  }
  else if(error.get_deg() < -180) {
    return error.get_rad() + 360 * DEG_TO_RAD;
  }
  return error.get_rad();
}



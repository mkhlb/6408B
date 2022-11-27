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
  Angle angle = Angle::from_rad(atan(y / x));
  Angle out;

  if (x < 0) {
    if (y < 0) {
      out.set_deg(180.0 + angle.get_deg());
    } else {
      out.set_deg(180.0 + angle.get_deg());
    }
  } else {
    if (y < 0) {
      out.set_deg(360.0 + angle.get_deg());
    } else {
      out.set_rad(angle.get_rad());
    }
  }

  return out;
}
Vector2 Vector2::get_normalized() {
  return Vector2(x / get_magnitude(), y / get_magnitude());
}
void Vector2::set_magnitude(double value) {
  double mag = get_magnitude();

  x = x / mag * value;

  y = y / mag * value;
}
void Vector2::set_angle_direction(double value) {
  double mag = get_magnitude();

  x = mag * cos(value);

  y = mag * sin(value);
}

double Angle::shortest_error(Angle from,
                            Angle to) // returns shortest error from angle this
                                      // method is being called on to other
{
  if (to.get_deg() - from.get_deg() <= 180 &&
      to.get_deg() - from.get_deg() >= -180) {
    return to.get_deg() - from.get_deg();
  } else // need to go the other way around
  {
    if (from.get_deg() > 180) {
      return 360 - from.get_deg() + to.get_deg();
    } else {
      return - from.get_deg() + to.get_deg() - 360;
    }
  }
}
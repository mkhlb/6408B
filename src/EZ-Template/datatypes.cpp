#include "EZ-Template/datatypes.hpp"

#include <math.h>

using namespace ez;

double Vector2::GetMagnitude() { return sqrt(x * x + y * y); }
Angle Vector2::GetAngleDirection() {
  Angle angle = Angle::FromRad(atan(y / x));
  Angle out;

  if (x < 0) {
    if (y < 0) {
      out.SetDeg(180.0 + angle.GetDeg());
    } else {
      out.SetDeg(180.0 + angle.GetDeg());
    }
  } else {
    if (y < 0) {
      out.SetDeg(360.0 + angle.GetDeg());
    } else {
      out.SetRad(angle.GetRad());
    }
  }

  return out;
}
Vector2 Vector2::GetNormalized() {
  return Vector2(x / GetMagnitude(), y / GetMagnitude());
}
void Vector2::SetMagnitude(double value) {
  double mag = GetMagnitude();

  x = x / mag * value;

  y = y / mag * value;
}
void Vector2::SetAngleDirection(double value) {
  double mag = GetMagnitude();

  x = mag * cos(value);

  y = mag * sin(value);
}

double Angle::ShortestError(Angle from,
                            Angle to) // returns shortest error from angle this
                                      // method is being called on to other
{
  if (to.GetDeg() - from.GetDeg() <= 180 &&
      to.GetDeg() - from.GetDeg() >= -180) {
    return to.GetDeg() - from.GetDeg();
  } else // need to go the other way around
  {
    if (from.GetDeg() > 180) {
      return 360 - from.GetDeg() + to.GetDeg();
    } else {
      return -from.GetDeg() + to.GetDeg() - 360;
    }
  }
}
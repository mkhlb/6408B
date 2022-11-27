#pragma once

#include "EZ-Template/util.hpp"

using namespace ez;

class Angle {
public:
  Angle() {
    _deg = 0;
    _rad = 0;
  }

  //Angle(double value) { SetDeg(value); }

  static Angle FromDegrees(double deg) {Angle a; a.SetDeg(deg); return a;}
  static Angle FromRad(double rad) {Angle a; a.SetRad(rad); return a;}

  void SetDeg(double value) {
    _deg = value;
    _rad = 3.1415926 / 180 * value;

    if (_deg >= 360.0f) {
      SetDeg(_deg - (floor(_deg / 360.0f) * 360));
    }

    if (_deg < 0) {
      SetDeg(360 + _deg - (ceil(_deg / 360.0f) * 360));
    }
  }
  void SetRad(double value) {
    _rad = value;
    _deg = 180 / 3.1415926 * value;

    if (_deg >= 360.0f) {
      SetDeg(_deg - (floor(_deg / 360.0f) * 360));
    }

    if (_deg < 0) {
      SetDeg(360 + _deg - (ceil(_deg / 360.0f) * 360));
    }
  }
  double GetDeg() { return _deg; }
  double GetRad() { return _rad; }

  static double ShortestError(Angle from, Angle to);

private:
  double _deg;
  double _rad;
};

class Vector2 {
public:
  Vector2(double xValue=0, double yValue=0) {
    x = xValue;
    y = yValue;
  }

  static Vector2 FromPolar(double magnitude, Angle angle) {
    return Vector2(magnitude * cos(angle.GetRad()), magnitude * sin(angle.GetRad()));
  }

  double x;
  double y;

  Vector2 GetNormalized();

  double GetMagnitude();
  Angle GetAngleDirection();

  void SetMagnitude(double value); // sets magnitude while preserving direction
  void SetAngleDirection(
      double value); // like rotating a vector, changes its polar
                     // coordinate angle while preserving magnitude

  double Dot(Vector2 obj) { return x * obj.x + y * obj.x; }

  Vector2 operator*(double const &obj) { return Vector2(x * obj, y * obj); }

  double operator*(Vector2 const &obj) {return x * obj.x + y * obj.y;}

  Vector2 operator+(Vector2 const &obj) {
    return Vector2(x + obj.x, y + obj.y);
  }

  Vector2 operator-(Vector2 const &obj) {
    return Vector2(x - obj.x, y - obj.y);
  }

  // private:
};
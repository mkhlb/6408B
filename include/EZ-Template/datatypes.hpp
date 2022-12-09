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

  static Angle from_degrees(double deg) {Angle a; a.set_deg(deg); return a;}
  static Angle from_rad(double rad) {Angle a; a.set_rad(rad); return a;}

  void set_deg(double value) {
    _deg = value;
    _rad = 3.1415926 / 180 * value;

    if (_deg >= 360.0f) {
      set_deg(_deg - (floor(_deg / 360.0f) * 360));
    }

    if (_deg < 0) {
      set_deg(360 + _deg - (ceil(_deg / 360.0f) * 360));
    }
  }
  void set_rad(double value) {
    _rad = value;
    _deg = 180 / 3.1415926 * value;

    if (_deg >= 360.0f) {
      set_deg(_deg - (floor(_deg / 360.0f) * 360));
    }

    if (_deg < 0) {
      set_deg(360 + _deg - (ceil(_deg / 360.0f) * 360));
    }
  }
  double get_deg() { return _deg; }
  double get_rad() { return _rad; }

  static Angle shortest_error(Angle from, Angle to);

  Angle operator+(const Angle &obj) {return from_rad(_rad + obj._rad);}
  Angle operator-(const Angle &obj) {return from_rad(_rad - obj._rad);}


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

  static Vector2 from_polar(double magnitude, Angle angle) {
    return Vector2(magnitude * cos(angle.get_rad()), magnitude * sin(angle.get_rad()));
  }

  double x;
  double y;

  Vector2 get_normalized();

  double get_magnitude();
  Angle get_angle_direction();

  void set_magnitude(double value); // sets magnitude while preserving direction
  void set_angle_direction(
      Angle value); // like rotating a vector, changes its polar
                     // coordinate angle while preserving magnitude

  double dot(Vector2 obj) { return x * obj.x + y * obj.x; }

  

  Vector2 operator-(Vector2 const &obj) {
    return Vector2(x - obj.x, y - obj.y);
  }

  // private:
};

double operator*(Vector2 const &obj1, Vector2 const &obj2);

Vector2 operator+(Vector2 const &obj1, Vector2 const &obj2);

Vector2 operator*(double const &scal, Vector2 const &vec);

Vector2 operator*(Vector2 const &vec, double const &scal);
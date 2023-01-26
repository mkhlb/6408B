#pragma once

#include "EZ-Template/util.hpp"

using namespace ez;

class Angle {
public:

  static constexpr double PI = 3.141592653589793238;
  static constexpr double RAD_TO_DEG = 180 / PI;
  static constexpr double DEG_TO_RAD = PI / 180;

  Angle() {
    _deg = 0;
    _rad = 0;
  }

  Angle(const Angle &copy) {
    _deg = copy._deg;
    _rad = copy._rad;
  }

  //Angle(double value) { SetDeg(value); }

  static Angle from_deg(double deg) {Angle a; a.set_deg(deg); return a;}
  static Angle from_rad(double rad) {Angle a; a.set_rad(rad); return a;}

  void set_deg(double value) {
    _deg = value;
    _rad = value * DEG_TO_RAD;

    if (_deg >= 360.0) {
      set_deg(_deg - (floor(_deg / 360.0) * 360));
    }

    if (_deg < 0) {
      set_deg(360.0 + _deg - (ceil(_deg / 360.0) * 360));
    }
  }
  void set_rad(double value) {
    _rad = value;
    _deg = value * RAD_TO_DEG;

    if (_deg >= 360.0) {
      set_deg(_deg - (floor(_deg / 360.0) * 360));
    }

    if (_deg < 0) {
      set_deg(360 + _deg - (ceil(_deg / 360.0) * 360));
    }
  }
  double get_deg() { return _deg; }
  double get_rad() { return _rad; }

  static double shortest_error(Angle from, Angle to);

  Angle operator+(const Angle &obj) {return from_rad(_rad + obj._rad);}
  Angle operator-(const Angle &obj) {return from_rad(_rad - obj._rad);}
  Angle operator-() {return from_rad(-_rad);}


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

  Vector2(const Vector2 &copy) {
    x = copy.x;
    y = copy.y;
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

struct PathPoint {
  Vector2 position;
  double advance_distance = -1;
  double lookahead = -1;
  double speed = -1;
  bool toggle_pid = false;

  PathPoint(Vector2 pos, double advance=-1, double lookahead=-1, double speed=-1, bool pid=false)
  :position(pos),
  advance_distance(advance),
  lookahead(lookahead),
  speed(speed),
  toggle_pid(pid)
  {

  }

  PathPoint(Vector2* pos, double advance=-1, double lookahead=-1, double speed=-1, bool pid=false)
  :position(*pos),
  advance_distance(advance),
  lookahead(lookahead),
  speed(speed),
  toggle_pid(pid)
  {

  }
};

// PathPoint operator+(PathPoint point, Vector2 vec) {return PathPoint(point.position + vec, point.advance_distance, point.lookahead, point.speed, point.toggle_pid);}

// PathPoint operator-(PathPoint point, Vector2 vec) {return PathPoint(point.position - vec, point.advance_distance, point.lookahead, point.speed, point.toggle_pid);}

// std::list<PathPoint> operator+(std::list<PathPoint> const &path_list, Vector2 const &offset) {
//   std::list<PathPoint> ret;
//   for(auto it = path_list.begin(); it != path_list.end(); ++it) {
//     ret.push_back(*it + offset);
//   }
//   return ret;
// }

// std::list<PathPoint> operator-(std::list<PathPoint> const &path_list, Vector2 const &offset) {
//   std::list<PathPoint> ret;
//   for(auto it = path_list.begin(); it != path_list.end(); ++it) {
//     ret.push_back(*it - offset);
//   }
//   return ret;
// }
#include "points.hpp"
#include "paths.hpp"
#include "EZ-Template/datatypes.hpp"

std::list<PathPoint> skills_second_roller_path = {
    {Vector2(40, -23), 8.0},
    {Vector2(14.5, -27), 14} // in front of roller
};

std::list<PathPoint> skills_first_shot_path = {
    {Vector2(16, -30), 4},
    {Vector2(13, -65), 12},
    {Vector2(11.5, -88), 16}, // in position for shot
};

std::list<PathPoint> skills_far_low_goal_horizontal_line_path = {
    {Vector2(22, -87), 12},
    {Vector2(50, -85), 8},
    {Vector2(43, -78), 9},
    {Vector2(35, -82), 14}, // in position for shot
};

std::list<PathPoint> skills_far_low_goal_lateral_line_path {
    {Vector2(55, -84), 7},
    {Vector2(54, -100), 16},
    {Vector2(53, -119), 19},
    {Vector2(55, -123), 8},
    {Vector2(69, -129), 8},
    {Vector2(53, -128), 15},
};

std::list<PathPoint> skills_near_line_path {
    {Vector2(30, -56), 8}, // first disc is at 36, -60
    {Vector2(59, -92), 28}, // third disc is at 60, -84
    {Vector2(60, -80), 14}, // in position to fire w/ run up
};

std::list<Vector2> far_goal_left_firing_path = {
    Vector2(58, -80),
    Vector2(11, -72),
    Vector2(14, -95),

};

std::list<Vector2> far_goal_right_firing_path = {

};
#include "points.hpp"
#include "paths.hpp"
#include "EZ-Template/datatypes.hpp"

std::list<PathPoint> skills_second_roller_path = {
    {Vector2(40, -23), 8},
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

std::list<PathPoint> skills_near_low_goal_horizontal_line_path = {
    {Vector2(120, -54), 12},
    {Vector2(92, -55), 8},
    {Vector2(99, -64), 9},
    {Vector2(107, -59), 14}, // in position for shot
};

std::list<PathPoint> skills_far_low_goal_lateral_line_path {
    {Vector2(55, -84), 7},
    {Vector2(53, -100), 16},
    {Vector2(52, -119), 19},
    {Vector2(55, -123), 8},
    {Vector2(69, -127), 8},
    {Vector2(52, -128.5), 15}, // in position for shot
};

std::list<PathPoint> skills_near_line_path {
    {Vector2(30, -56), 8}, // first disc is at 36, -60
    {Vector2(59, -92), 28}, // third disc is at 60, -84
    {Vector2(60, -80), 14}, // in position to fire w/ run up
};

std::list<PathPoint> skills_far_corner_triple_stack_path { // tripls stack sitting at about 108, -108
    {Vector2(108, -108), 10},
    {Vector2(104, -94), 10},
    {Vector2(117, -105), 10},
    {Vector2(120, -110), 14},
    {Vector2(127, -111), 14},
};

std::list<PathPoint> skills_near_goal_first_shot_path {
    {Vector2(126, -112), 4},
    {Vector2(129, -67), 12},
    {Vector2(130.5, -53), 16}, // in position for shot
};

std::list<Vector2> far_goal_left_firing_path = {
    Vector2(58, -80),
    Vector2(11, -72),
    Vector2(14, -95),
};

std::list<Vector2> far_goal_right_firing_path = {

};
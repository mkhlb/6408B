#include "points.hpp"
#include "paths.hpp"
#include "EZ-Template/datatypes.hpp"
#include <list>

std::list<PathPoint> skills_second_roller_path = {
    {Vector2(40, -22), 8},
    {Vector2(13.5, -30), 12} // in front of roller
};

std::list<PathPoint> skills_first_shot_path = {
    {Vector2(16, -34), 6},
    {Vector2(14.5, -65), -1},
    {Vector2(15, -80.5), -1, -1, -1, false}, // in position for shot
};

std::list<PathPoint> skills_far_low_goal_horizontal_line_path = {
    {Vector2(22, -84), 12},
    {Vector2(50, -86), -1},
    {Vector2(34, -74), -1},
    {Vector2(30, -71), 14}, // in position for shot
};

std::list<PathPoint> skills_near_line_path {
    {Vector2(36, -50), 10}, // first disc is at 36, -60
    {Vector2(57.5, -80), -1}, // third disc is at 60, -84
    // {Vector2(57.5, -80), 14}, // in position to fire w/ run up
};

std::list<PathPoint> skills_far_low_goal_lateral_line_path {
    {Vector2(55, -84), 7},
    {Vector2(53, -100), 16},
    {Vector2(52, -119), 19},
    {Vector2(55, -123), 8},
    {Vector2(69, -127), 8},
    {Vector2(52, -128.5), 15}, // in position for shot
};

std::list<PathPoint> skills_near_low_goal_horizontal_line_path = {
    {Vector2(120, -54), 12},
    {Vector2(92, -55), 8},
    {Vector2(99, -61), 13},
    {Vector2(107, -58), 14}, // in position for shot
};

std::list<PathPoint> skills_near_low_goal_lateral_line_path {
    {Vector2(85, -58), 7},
    {Vector2(88, -42), 16},
    {Vector2(89, -33), 19},
    {Vector2(86, -19), 8},
    {Vector2(72, -13), 8},
    {Vector2(90, -12.5), 7}, // in position for shot
    //{Vector2(124.5, -19.5), 60} // drive a bit towards the goal to aim while moving
};



std::list<PathPoint> skills_far_line_path {
    {Vector2(80, -58), 10}, // first disc at 83, -59
    {Vector2(97, -69), 15}, // second disc at 95, -71
    {Vector2(107, -83), 14}, // final disc is at 107, -83
    {Vector2(107, -58), 15},
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
    {Vector2(128, -67), 12},
    {Vector2(129, -52), 16}, // in position for shot
};

std::list<Vector2> far_goal_left_firing_path = {
    Vector2(58, -80),
    Vector2(11, -72),
    Vector2(14, -95),
};

std::list<Vector2> far_goal_right_firing_path = {
    Vector2(62, -84),
    Vector2(70, -131),
    Vector2(47, -128),
};
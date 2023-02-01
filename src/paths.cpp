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
    {Vector2(16.5, -81), -1, -1, -1, false}, // in position for shot
};

std::list<PathPoint> skills_far_low_goal_horizontal_line_path = {
    {Vector2(26.8, -88.2), 12},
    {Vector2(55, -93), -1},
    {Vector2(34, -74.5), -1},
    {Vector2(30, -75.0), 14}, // in position for shot
};

std::list<PathPoint> skills_near_line_path {
    {Vector2(36, -56), 10, -1, -1, true}, // first disc is at 34.86, -58.42
    {Vector2(57, -81.0), -1}, // third disc is at 58.42, -81.99
    // {Vector2(57.5, -80), 14}, // in position to fire w/ run up
};

std::list<PathPoint> skills_far_low_goal_lateral_line_path {
    {Vector2(72, -84), -1},
    {Vector2(57, -99), -1},
    {Vector2(49.5, -115), -1},
    {Vector2(48, -123), -1},
    {Vector2(50.5, -133), -1}, // in position for shot
};

std::list<PathPoint> skills_far_middle_triple_stack_path {
    PathPoint(Vector2(61.5, -128.5)),
    PathPoint(Vector2(75, -113)),
    PathPoint(Vector2(81.99, -98.0), 22), // triple stack at 81.99, -105.35
};

std::list<PathPoint> skills_far_corner_triple_stack_path { // tripls stack sitting at about 105.35, -105.35
    {Vector2(108, -108), 10},
    {Vector2(104, -94), 10},
    {Vector2(117, -105), 10},
    {Vector2(113, -103), 14},
    
    {Vector2(122, -106), 14},
};

std::list<PathPoint> skills_near_goal_first_shot_path {
    {Vector2(124.4, -106.4), 4},
    {Vector2(125.9, -75.4), -1},
    {Vector2(123.9, -61.4), -1}, // in position for shot
};

std::list<PathPoint> skills_near_low_goal_horizontal_line_path = {
    {Vector2(120, -54), 12},
    {Vector2(92, -55), 8},
    {Vector2(99, -61), 13},
    {Vector2(107, -58), 14}, // in position for shot
};

std::list<PathPoint> skills_far_line_path {
    {Vector2(80, -58), 10}, // first disc at 81.99, -58.42
    {Vector2(97, -69), 15}, // second disc at 93.77, -70.20
    {Vector2(107, -83), 14}, // final disc is at 105.35, -81.99
    {Vector2(107, -58), 15},
};

std::list<PathPoint> skills_near_low_goal_lateral_line_path {
    {Vector2(85, -58), 7},
    {Vector2(88, -42), 16},
    {Vector2(89, -33), 19},
    {Vector2(86, -19), 12},
    {Vector2(72, -13), 12},
    {Vector2(90, -12.5), 12}, // in position for shot
    //{Vector2(124.5, -19.5), 60} // drive a bit towards the goal to aim while moving
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
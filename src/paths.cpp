#include "points.hpp"
#include "paths.hpp"
#include "EZ-Template/datatypes.hpp"
#include <list>

std::list<PathPoint> skills_second_roller_path = {
    {Vector2(44, -19), 8},
    {Vector2(13.5, -30), 12} // in front of roller
};

std::list<PathPoint> skills_first_shot_path = {
    {Vector2(16, -34), 6},
    {Vector2(14.0, -65), -1},
    {Vector2(15.8, -81), -1, -1, -1, false}, // in position for shot
};

std::list<PathPoint> skills_far_low_goal_horizontal_line_path = {
    {Vector2(26.8, -85.2), 12},
    {Vector2(55, -90), -1},
    {Vector2(34, -74.5), -1},
    {Vector2(30, -75.0), 14}, // in position for shot
};

std::list<PathPoint> skills_near_line_path {
    {Vector2(36, -56), 10, -1, -1, true}, // first disc is at 34.86, -58.42
    {Vector2(56, -80.0), -1}, // third disc is at 58.42, -81.99
    // {Vector2(57.5, -80), 14}, // in position to fire w/ run up
};

std::list<PathPoint> transposed_skills_far_line_path {
    {Vector2(36,-56), 10, -1, -1, true},
    {Vector2(57, -79.5), -1},
};

std::list<PathPoint> skills_far_low_goal_lateral_line_path {
    {Vector2(74.5, -84), -1},
    {Vector2(57, -99), -1},
    {Vector2(49.5, -115), -1},
    {Vector2(48, -123), -1},
    {Vector2(53.0, -132), -1}, // in position for shot
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
    {Vector2(115, -108), 14},
    {Vector2(126, -110), 14},
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







// PREMATCH PATHS

std::list<PathPoint> win_point_first_shot_path = {
    {Vector2(35, -25)},
    {Vector2(58.5, -22)},
    {Vector2(70, -27)},
    {Vector2(73, -44), 10}
};

std::list<PathPoint> win_point_second_shot_path = {
    {Vector2(83, -61)},
    {Vector2(87, -60)}
};

std::list<PathPoint> win_point_third_shot_path = {
    {Vector2(94, -74)},
    {Vector2(100.5, -71)}
};

std::list<PathPoint> win_point_fourth_shot_path = {
    {Vector2(110, -86)},
    {Vector2(115, -84)}
};

std::list<PathPoint> win_point_roller_path = {
    {Vector2(118, -105)},
    {Vector2(126, -113)},
};

std::list<PathPoint> prematch_far_roller_path = {
    {Vector2(118, -95)},
    {Vector2(120, -105)},
    {Vector2(122, -110)},
};

std::list<PathPoint> prematch_far_third_shot_path = {
    {Vector2(94, -65)},
    {Vector2(105.5, -57)}
};

std::list<PathPoint> prematch_near_second_shot_path = {
    {Vector2(74, -46)},
    {Vector2(81, -46), 8},
    {Vector2(75, -46)}
};

std::list<PathPoint> prematch_near_third_shot_path = {
    {Vector2(74, -40)},
    {Vector2(81, -35), 8},
    {Vector2(75, -46)}
};

std::list<PathPoint> prematch_near_fourth_shot_path = {
    {Vector2(75, -30)},
    {Vector2(81, -24), 8},
    {Vector2(77, -42)}
};
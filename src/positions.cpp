#include "positions.hpp"
#include "EZ-Template/datatypes.hpp"

Vector2 far_goal = Vector2(17.5, -122.5);
Vector2 far_goal_left_firing_spot = Vector2(18, -80); // field is about 141
Vector2 far_goal_right_firing_spot = Vector2(64, -124);
Vector2 near_lateral_roller = Vector2(12, -29.5); //middle wheel is at 13.5
Vector2 near_horizontal_roller = Vector2(29.5, -12);
Vector2 far_lateral_roller = Vector2(132, -114.5);
Vector2 far_horizontal_roller = Vector2(112.5, -132);

Vector2 skills_start = Vector2(32.45, -12.2); // middle encoder at abt 13.5 y
Vector2 skills_second_roller = Vector2(14.5, -30);
std::list<Vector2> skills_second_roller_path = {
    Vector2(42, -24),
    skills_second_roller
};

Vector2 skills_first_shot = Vector2(13, -89);
std::list<Vector2> skills_first_shot_path = {
    Vector2(20, -30),
    Vector2(13, -65),
    skills_first_shot,
};
Vector2 skills_second_shot = Vector2(32, -81);
Vector2 skills_third_shot = Vector2(40, -85);
Vector2 skills_fifth_shot = Vector2(57, -128); // similar to first shot

Vector2 skills_near_line_start = Vector2(36, -60);
Vector2 skills_near_line_end = Vector2(58, -80);

std::list<Vector2> far_goal_left_firing_path = {
    Vector2(58, -80),
    Vector2(11, -72),
    Vector2(14, -95),

};

std::list<Vector2> far_goal_right_firing_path = {

};

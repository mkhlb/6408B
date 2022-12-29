#pragma once

#include "EZ-Template/datatypes.hpp"
#include <list>

extern Vector2 far_goal;
extern Vector2 far_goal_left_firing_spot;
extern Vector2 far_goal_right_firing_spot;
extern Vector2 near_lateral_roller; //middle wheel is at 13.5
extern Vector2 near_horizontal_roller;
extern Vector2 far_lateral_roller;
extern Vector2 far_horizontal_roller;

extern Vector2 skills_start;

extern Vector2 skills_second_roller;
extern std::list<Vector2> skills_second_roller_path;
extern Vector2 skills_third_roller;
extern Vector2 skills_fourth_roller;

extern Vector2 skills_first_shot;
extern Vector2 skills_second_shot;
extern Vector2 skills_third_shot;
extern Vector2 skills_fourth_shot;
extern Vector2 skills_fifth_shot;

extern Vector2 skills_near_line_start;
extern Vector2 skills_near_line_end;

extern std::list<Vector2> far_goal_left_firing_path;
extern std::list<Vector2> far_goal_right_firing_path;
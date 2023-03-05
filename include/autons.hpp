#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "autons.hpp"
#include "cata_intake.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

extern Drive chassis;
extern pros::ADIDigitalOut expansion;
extern pros::ADIDigitalOut boost;
extern mkhlib::CatapultIntakeController cata_intake;

void drive_test();
void turn_test();
void point_drive_test();
void point_turn_test();
void odom_test();
void path_test();
void heading_test();
void skills();
void skills1();
void skills2();

void prematch_win_point();
void prematch_near();
void prematch_far();
void prematch_near_roller();
void prematch_far_roller();
void null();

void default_constants();
void one_mogo_constants();
void two_mogo_constants();
void exit_condition_defaults();
void exit_condition_hit_wall();
void modified_exit_condition();

void exit_condition_defaults();
void chasing_heading_constants();
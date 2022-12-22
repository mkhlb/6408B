#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "cata_intake.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

extern Drive chassis;
extern mkhlib::CatapultIntakeController cata_intake;

void roll_test();
void swing_test();
void drive_test();

void default_constants();
void one_mogo_constants();
void two_mogo_constants();
void exit_condition_defaults();
void exit_condition_hit_wall();
void modified_exit_condition();
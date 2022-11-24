#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "cata_intake.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

extern Drive chassis;
extern mkhlib::CatapultIntakeController cata_intake;
extern pros::ADIDigitalOut poonamic;

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void combining_movements();
void interfered_example();
void intake_and_drive();
void cata_swing();
void winpoint_half();
void winpoint_full();
void auto_skills();
void farside_roller(); 

void roll_test();

void default_constants();
void one_mogo_constants();
void two_mogo_constants();
void exit_condition_defaults();
void modified_exit_condition();
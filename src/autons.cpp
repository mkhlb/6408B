#include "autons.hpp"
#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"

#include "pros/rtos.hpp"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

const int DRIVE_SPEED =
    110; // This is 110/127 (around 87% of max speed).  We don't suggest making
         // this 127. If this is 127 and the robot tries to heading correct,
         // it's only correcting by making one side slower.  When this is 87%,
         // it's correcting by making one side faster and one side slower,
         // giving better heading correction.
const int TURN_SPEED = 110;
const int SWING_SPEED = 110;
const int INTK_IN = 0.9*200*84/36;

///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier
// game objects, or with lifts up vs down. If the objects are light or the cog
// doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 9, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
  //cata_intake.roller_set_pid_constants(.375, 0.009, 3.75, 10);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
  //cata_intake.roller_set_exit_condition(100, 5, 500, 30, 500, 500);
}

void exit_condition_early_drive() { // made to exit the drive way earlier, more error so only use with position tracking or if planning on recording error, made when distance forward and backwards doesn't matter too much
  chassis.set_exit_condition(chassis.drive_exit, 50, 6, 500, 10, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}


void exit_condition_hit_wall() { // made to exit as soon as velocity is 0, for detecting crashing into objects
  chassis.set_exit_condition(chassis.drive_exit, 100, 3, 500, 7, 30, 10);
}

// 6408B code
// __________________________________________________________________________________________________________________________________________________________________________________________________________

void roll(double max_dist, double speed, double roll_amount) // roll_amount is degrees
{
  exit_condition_hit_wall(); // set exit conditions to conditions very sensitive to interference
  chassis.set_drive_pid(max_dist, speed); // drive forward 7 inches, or until meeting resistance
  chassis.wait_drive(); // wait until drive exits
  exit_condition_defaults(); //reset exit conditions
  chassis.set_drive_pid(-1, speed);

  cata_intake.roller_pid_move(roll_amount, 127);
  cata_intake.wait_roller();
  cata_intake.roller_velocity(0);
}

void roll_time(double max_dist, double back_distance, double speed, double roll_time) { //roll time can be negative or positive
  exit_condition_hit_wall(); // set exit conditions to conditions very sensitive to interference
  chassis.set_drive_pid(max_dist, speed); // drive forward 7 inches, or until meeting resistance
  chassis.wait_drive(); // wait until drive exits
  exit_condition_defaults(); //reset exit conditions
  chassis.set_drive_pid(back_distance, speed);

  cata_intake.roller_time(fabs(roll_time), 200 * util::sgn(roll_time));
  cata_intake.wait_roller();
}



//TEST OF VERY SENSITIVE EXIT CONDITIONS: Robot will exit almost immediately after the velocity of the wheel is 0
void roll_test() {
  roll(20, 115, -180);
  chassis.set_drive_pid(-10, DRIVE_SPEED);
  chassis.wait_drive();
  roll_time(20, -.4, 115, 500);
  chassis.set_drive_pid(-10, DRIVE_SPEED);
  chassis.wait_drive();
}

void skills() {
  default_constants();
  chassis.reset_position(Vector2(32, 19.5), Angle::from_deg(90));
  //ROBOT TO GOAL: -15, 102
  Vector2 far_goal = Vector2(17, 121.5);
  cata_intake.cata_prime();
  roll_time(30, -.4, 50, 280);
  chassis.set_heading_relative_swing_pid(ez::RIGHT_SWING, 65, SWING_SPEED, .3);
  chassis.wait_until_heading_relative(25);
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(-136), SWING_SPEED, -.5);
  chassis.wait_drive();
  chassis.set_drive_pid(20, DRIVE_SPEED * .8);
  chassis.wait_until(6.5);
  chassis.set_orientation_swing_pid(ez::RIGHT_SWING, Angle::from_deg(180), SWING_SPEED, .24);
  chassis.wait_drive();
  chassis.set_drive_pid(30, DRIVE_SPEED);
  chassis.wait_until(3);
  cata_intake.intake_stop();
  roll_time(30, -.9, 50, 280);
  chassis.reset_position(Vector2(19.5, 29.5), chassis.orientation);
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(84), SWING_SPEED, -.2);
  chassis.wait_until_orientation(Angle::from_deg(155));
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_offside_multiplier(.4);
  chassis.wait_until_orientation(Angle::from_deg(105));
  chassis.set_offside_multiplier(.6);
  // chassis.wait_until_orientation(Angle::from_deg(-90));
  // chassis.set_orientation_turn_pid(Angle::from_deg(-90), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-49, DRIVE_SPEED);
  chassis.wait_drive();
  cata_intake.intake_stop();
  //chassis.set_point_turn_pid(far_goal, TURN_SPEED, Angle::from_deg(180));
  chassis.set_orientation_turn_pid(Angle::from_deg(95), TURN_SPEED);
  chassis.wait_drive();
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  chassis.set_orientation_turn_pid(Angle::from_deg(-10), TURN_SPEED);
  chassis.wait_drive();
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_drive_pid(50, 70);
  chassis.wait_until(40);
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(42), SWING_SPEED, -.3);
  chassis.wait_until_orientation(Angle::from_deg(34));
  cata_intake.intake_velocity(-.8 * INTK_IN);
  chassis.wait_drive();
  pros::delay(60);
  chassis.set_heading_relative_turn_pid(-7, TURN_SPEED);
  chassis.set_drive_pid(-6, DRIVE_SPEED);
  chassis.wait_until(-4);
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_orientation_turn_pid(Angle::from_deg(135), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(75), SWING_SPEED, .2);
  chassis.wait_drive();
  chassis.set_drive_pid(-28, DRIVE_SPEED);
  chassis.wait_drive();
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  chassis.set_orientation_swing_pid(ez::RIGHT_SWING, Angle::from_deg(-100), SWING_SPEED, .4);
  chassis.wait_drive();
  chassis.set_drive_pid(45, DRIVE_SPEED);
  chassis.wait_until(39);
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(-10), SWING_SPEED);
  chassis.wait_drive();
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
}

void turn_test() {
  chassis.set_point_turn_pid(Vector2(), 80);
}

void swing_test() {
  chassis.set_drive_pid(3, 110);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED, .4);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED, .7);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::RIGHT_SWING, 45, SWING_SPEED, -.5);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, -45, SWING_SPEED, -.3);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED, -.3);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::RIGHT_SWING, 90, SWING_SPEED, -.5);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED, .7);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 0, SWING_SPEED, .4);
  chassis.wait_drive();
  chassis.set_drive_pid(-3, 110);
  chassis.wait_drive();

}



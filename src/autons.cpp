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
  chassis.set_pid_constants(&chassis.headingPID, 8.7, 0, 22, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5.1, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
  cata_intake.roller_set_pid_constants(3, 0.000, 7, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 1000, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 1.5, 500, 7, 1000, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 1000, 500);
  cata_intake.roller_set_exit_condition(100, 10, 500, 50, 1000, 500);
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

void roll(double max_dist, double back_distance, double speed, double roll_amount) // roll_amount is degrees
{
  exit_condition_hit_wall(); // set exit conditions to conditions very sensitive to interference
  chassis.set_drive_pid(max_dist, speed); // drive forward 7 inches, or until meeting resistance
  chassis.wait_drive(); // wait until drive exits
  exit_condition_defaults(); //reset exit conditions
  chassis.set_drive_pid(back_distance, speed);

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
  default_constants();
  exit_condition_defaults();
  roll(30, -.4, 50, 180);
  // cata_intake.roller_pid_move(-180, 110);
  // cata_intake.wait_roller();
  // cata_intake.roller_pid_move(360, 110);
  // cata_intake.wait_roller();
}

void skills() {
  default_constants();
  chassis.reset_position(Vector2(32, -11.5), Angle::from_deg(90));
  //ROBOT TO GOAL: -15, 102
  Vector2 far_goal = Vector2(18.5, -139.5);
  Vector2 first_shot = Vector2(14, -88.5);
  cata_intake.cata_prime();
  roll_time(30, -.5, 50, 280);
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
  roll_time(30, -.11, 50, 280);
  chassis.set_mode(ez::DISABLE);
  pros::delay(500);
  //chassis.reset_position(Vector2(9.75, chassis.position.y), chassis.orientation);
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(80), SWING_SPEED, -.2);
  chassis.wait_until_orientation(Angle::from_deg(155));
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_heading_relative_swing_pid(ez::LEFT_SWING, chassis.error_to_point(first_shot, Angle::from_deg(180)), SWING_SPEED);
  // chassis.wait_until_orientation(Angle::from_deg(-90));
  // chassis.set_orientation_turn_pid(Angle::from_deg(-90), TURN_SPEED);
  chassis.wait_drive();
  //chassis.set_drive_pid(-49, DRIVE_SPEED);
  chassis.set_heading_relative_turn_pid(chassis.error_to_point(first_shot, Angle::from_deg(180)), TURN_SPEED);
  chassis.set_straight_point_drive_pid(first_shot, DRIVE_SPEED);
  // chassis.wait_until(-15);
  // chassis.set_heading_relative_turn_pid(chassis.error_to_point(first_shot, Angle::from_deg(180)), TURN_SPEED);
  // chassis.set_straight_point_drive_pid(first_shot, DRIVE_SPEED);
  chassis.wait_drive();
  cata_intake.intake_stop();
  //chassis.set_point_turn_pid(far_goal, TURN_SPEED, Angle::from_deg(180));
  chassis.set_orientation_turn_pid(Angle::from_deg(95), TURN_SPEED);
  chassis.wait_drive();
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  chassis.set_straight_point_drive_pid(Vector2(0, -92), DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(-4), SWING_SPEED, -.5);
  chassis.wait_drive();
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_drive_pid(50, 70);
  chassis.wait_until(40);
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(38), SWING_SPEED, -.3);
  chassis.wait_until_orientation(Angle::from_deg(32));
  cata_intake.intake_velocity(-.8 * INTK_IN);
  chassis.wait_drive();
  pros::delay(130);
  //chassis.set_heading_relative_turn_pid(-7, TURN_SPEED);
  chassis.set_point_turn_pid(far_goal, TURN_SPEED, Angle::from_deg(180));
  chassis.wait_drive();
  chassis.set_drive_pid(-8, DRIVE_SPEED);
  chassis.wait_drive();
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_orientation_turn_pid(Angle::from_deg(135), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(82), SWING_SPEED, .2);
  chassis.wait_drive();
  chassis.set_drive_pid(-30, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_point_turn_pid(far_goal, TURN_SPEED, Angle::from_deg(180));
  chassis.wait_drive();
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  chassis.set_orientation_turn_pid(Angle::from_deg(74), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_target_relative_swing_pid(ez::RIGHT_SWING, -178, SWING_SPEED, .43);
  chassis.wait_drive();
  chassis.set_drive_pid(45, DRIVE_SPEED);
  chassis.wait_until(8);
  chassis.set_max_speed(DRIVE_SPEED * .45);
  chassis.wait_until(28);
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(-85), SWING_SPEED * .75, 0);
  chassis.wait_drive();
  chassis.set_drive_pid(40, DRIVE_SPEED * .55);
  chassis.wait_until(9);
  chassis.set_orientation_swing_pid(ez::LEFT_SWING, Angle::from_deg(-12), SWING_SPEED, .1);
  chassis.wait_drive();
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
}

void skills2() {
  default_constants();
  exit_condition_defaults();
  chassis.reset_position(Vector2(32, -11.5), Angle::from_deg(92));
  pros::delay(10); // Wait for odometry to use the reset orientation
  chassis.set_orientation_turn_pid(Angle::from_deg(90), TURN_SPEED); // Set heading PID
  cata_intake.cata_prime();
  roll(30, -.5, 50, 180); // First near roller
  chassis.set_orientation_swing_pid(ez::RIGHT_SWING, Angle::from_deg(180), SWING_SPEED); // Swing directly in front of inter-roller disc
  cata_intake.intake_velocity(INTK_IN);
  chassis.wait_drive();
  chassis.set_pid_constants(&chassis.swingPID, 7.3, 0, 43, 0); // Wide swing constants
  chassis.set_target_relative_swing_pid(ez::LEFT_SWING, 52, SWING_SPEED, .4);
  chassis.wait_drive(); // Swing into disc
  chassis.set_orientation_swing_pid(ez::RIGHT_SWING, Angle::from_deg(180), SWING_SPEED, .4);
  chassis.wait_drive(); // Swing into second near roller
  default_constants(); // Normal swing constants
  cata_intake.intake_stop();
  roll(30, -.8, 50, 180); // Second near roller
  chassis.set_target_relative_swing_pid(ez::LEFT_SWING, -95.5, SWING_SPEED); 
  chassis.wait_drive(); // Swing into far goal
  cata_intake.intake_velocity(INTK_IN); // Continue intaking in case discs aren't settled
  chassis.set_drive_pid(-59, DRIVE_SPEED, true); 
  chassis.wait_drive(); // Drive to far goal
  chassis.set_target_relative_turn_pid(15, TURN_SPEED);
  chassis.wait_drive(); // Turn for first shot
  cata_intake.cata_shoot(); // First shot
  cata_intake.wait_cata_done_shot();
  /*** FIRST SHOT DONE ***/
  chassis.set_orientation_turn_pid(Angle::from_deg(-7), TURN_SPEED); 
  chassis.wait_drive(); // Turn to slightly face low goal
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_drive_pid(20, DRIVE_SPEED * .8, true);
  chassis.wait_drive(); // Intake first two discs
  chassis.set_orientation_turn_pid(Angle::from_deg(0), TURN_SPEED * .8); 
  chassis.wait_drive(); // Turn to third disc
  chassis.set_drive_pid(17, DRIVE_SPEED * .6, true);
  chassis.wait_drive(); // Intake third disc
  chassis.set_orientation_turn_pid(Angle::from_deg(3), TURN_SPEED * .8);
  chassis.wait_drive(); // Face out a bit
  chassis.set_target_relative_swing_pid(ez::LEFT_SWING, 49, SWING_SPEED * .7);
  cata_intake.intake_time(700, INTK_IN); // Intake third disc, and probably extra one
  cata_intake.wait_intake(); 
  cata_intake.intake_velocity(-INTK_IN * .5); // Outtake extra disc
  chassis.wait_drive(); // Swing out to shot
  chassis.set_drive_pid(-6, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_target_relative_turn_pid(-3, TURN_SPEED);
  chassis.wait_drive(); // Position for second shot
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  /*** SECOND SHOT DONE ***/
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_target_relative_swing_pid(ez::RIGHT_SWING, -40, SWING_SPEED, .42);
  chassis.wait_drive(); // Swing into first disc
  chassis.set_orientation_turn_pid(Angle::from_deg(135), TURN_SPEED);
  chassis.wait_drive(); // Turn to line of discs
  chassis.set_drive_pid(30, DRIVE_SPEED * .7, true);
  chassis.wait_drive(); // Intake line of discs
  chassis.set_target_relative_swing_pid(ez::LEFT_SWING, -67.0, SWING_SPEED);
  chassis.wait_drive(); // Swing to third shot
  chassis.set_drive_pid(-22, DRIVE_SPEED, true);
  chassis.wait_drive(); // Drive to third shot
  chassis.set_target_relative_turn_pid(-3, TURN_SPEED);
  chassis.wait_drive(); // Aim for third shot
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  /*** THIRD SHOT DONE ***/
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



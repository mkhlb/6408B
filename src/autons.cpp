#include "autons.hpp"
#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"

#include "points.hpp"
#include "paths.hpp"
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
  chassis.set_pid_constants(&chassis.headingPID, 4.2, 0, 45, 0);
  chassis.set_pid_constants(&chassis.left_forward_drivePID, 0.51, 0.0016, 3.8, 600);
  chassis.set_pid_constants(&chassis.right_forward_drivePID, 0.51, 0.0016, 3.8, 600);
  chassis.set_pid_constants(&chassis.left_backward_drivePID, .75, 0, 4, 0);
  chassis.set_pid_constants(&chassis.right_backward_drivePID, .75, 0, 4, 0);
  chassis.set_pid_constants(&chassis.turnPID, 6.9, 0.0026, 50, 10);
  chassis.set_pid_constants(&chassis.swingPID, 6.8, 0, 50, 0);
  cata_intake.roller_set_pid_constants(5, 0.000, 14, 0);
}

void chasing_heading_constants() {
  chassis.set_pid_constants(&chassis.headingPID, 4.2, 0, 45, 0);
}

void wide_swing_constants() { // offside = .3
  chassis.set_pid_constants(&chassis.swingPID, 7.5, 0, 45, 0);
}

void very_wide_swing_constants() { // offside = .6
  chassis.set_pid_constants(&chassis.swingPID, 8.3, 0, 38, 0);
}

void narrow_swing_constants() { // offside = -.3
  chassis.set_pid_constants(&chassis.swingPID, 6.3, 0, 49, 0);
}

void very_narrow_swing_constants() { // offside = -.6
  chassis.set_pid_constants(&chassis.swingPID, 5.8, 0, 50.5, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 1000, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 1.5, 500, 7, 1000, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 1000, 500);
  chassis.headingPID.set_exit_condition(120, 3, 700, 7, 1000, 500);
  cata_intake.roller_set_exit_condition(100, 10, 500, 50, 2000, 3000);
}

void exit_condition_early_drive() { // made to exit the drive way earlier, more error so only use with position tracking or if planning on recording error, made when distance forward and backwards doesn't matter too much
  chassis.set_exit_condition(chassis.drive_exit, 50, 6, 500, 10, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 50000, 7, 50000, 50000);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 50000, 7, 50000, 50000);
  chassis.set_exit_condition(chassis.drive_exit, 80, 25, 30000, 150, 50000, 50000);
}


void exit_condition_hit_wall() { // made to exit as soon as velocity is 0, for detecting crashing into objects
  chassis.set_exit_condition(chassis.drive_exit, 100, 3, 500, 7, 30, 100);
}

// 6408B code
// __________________________________________________________________________________________________________________________________________________________________________________________________________

void roll(double max_dist, Angle target_orientation, double back_distance, double speed, double roll_amount) // roll_amount is degrees
{
  chassis.set_orientation_heading_pid(target_orientation);
  chassis.set_drive_pid(max_dist, speed); // drive forward 7 inches, or until meeting resistance
  pros::delay(200);
  exit_condition_hit_wall(); // set exit conditions to conditions very sensitive to interference
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
  chasing_heading_constants();
  chassis.reset_position(Vector2(10.5, -29.5), Angle::from_deg(180));
  pros::delay(10);
  chassis.set_orientation_turn_pid(Angle::from_deg(170), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_orientation_turn_pid(Angle::from_deg(180), TURN_SPEED);
  chassis.wait_drive();
  
  //chassis.drive_to_point(Vector2(20, -15), DRIVE_SPEED, true);
}

void path_test() {
  chassis.reset_path();
  chassis.add_point(Vector2(0,0), 16);
  chassis.add_point(Vector2(24, 0), 16);
  chassis.add_point(Vector2(48, -18), 16);
  chassis.add_point(Vector2(48, -30), 16);
  chassis.add_point(Vector2(24, -48), 16);
  chassis.add_point(Vector2(0, -48), 16);
  //chassis.drive_to_points(DRIVE_SPEED);
  //chassis.drive_to_points(DRIVE_SPEED);

  chassis.set_path_pid(DRIVE_SPEED, 18, ez::AGNOSTIC);
  chassis.wait_until_distance_travelled(10);
  chassis.set_max_speed(70);
  chassis.wait_until_absolute_points_passed(3);
  chassis.set_max_speed(110);
  chassis.wait_until_distance_remaining(24);
  chassis.set_max_speed(50);
  chassis.wait_drive();
  chassis.set_orientation_turn_pid(Angle::from_deg(0), TURN_SPEED);
  chassis.wait_drive();
  
}

void aim_and_fire_far_goal(Angle offset = Angle(), double runup = 0) {
  chassis.set_point_turn_pid(far_goal, TURN_SPEED, Angle::from_deg(180) + offset);
  chassis.wait_drive();
  if(runup != 0) {
    chassis.set_drive_pid(runup, DRIVE_SPEED);
    chassis.wait_until_distance_travelled(runup + 7);
  }
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  
}

void skills() {
  skills1();
  skills2();
}

void skills1() {
  default_constants();
  exit_condition_defaults();
  cata_intake.cata_prime();
  chassis.reset_position(skills_start, Angle::from_deg(91.5));
  chassis.set_heading_relative_heading_pid(0);
  
  chassis.wait_drive();
  roll(30, Angle::from_deg(91.5), -.5, 50, 200);

  chassis.set_path_pid(skills_second_roller_path, DRIVE_SPEED, 8, ez::BACKWARD, 0);
  chassis.wait_until_absolute_points_passed(1);
  cata_intake.intake_velocity(INTK_IN * .7);
  chassis.set_point_path_orientation(ez::FORWARD);
  chassis.wait_drive();
  cata_intake.intake_stop();
  roll(30, Angle::from_deg(181.5), -.6, 50, 200);
  // start driving towards first shot
  chassis.set_path_pid(skills_first_shot_path, DRIVE_SPEED, 14, ez::BACKWARD);
  chassis.wait_until_absolute_points_passed(1);
  chassis.set_path_lookahead(12);
  cata_intake.intake_velocity(INTK_IN);
  chassis.wait_drive();
  aim_and_fire_far_goal();
  chassis.set_orientation_turn_pid(Angle::from_deg(5), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_path_pid(skills_far_low_goal_horizontal_line_path, DRIVE_SPEED * .7, 14, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_max_speed(DRIVE_SPEED);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.wait_drive();
  aim_and_fire_far_goal(Angle::from_deg(-2)); // place a little to the left
  chassis.set_path_pid(skills_near_line_path, DRIVE_SPEED, 19, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  cata_intake.intake_velocity(INTK_IN * .8);
  chassis.wait_drive();
  aim_and_fire_far_goal(Angle::from_deg(-.75), -9.8);

  // chassis.set_point_drive_pid(skills_near_line_end + Vector2(-2, 2), DRIVE_SPEED * .7, ez::FORWARD);
  // chassis.set_orientation_turn_pid(Angle::from_deg(-90), TURN_SPEED);
  // chassis.wait_drive();
  // chassis.set_drive_pid(50, DRIVE_SPEED * .8);
  // chassis.wait_until_distance_travelled(38);
  // chassis.set_point_drive_pid(skills_fifth_shot, DRIVE_SPEED, ez::FORWARD);
  // aim_and_fire_far_goal();

  chassis.set_path_pid(skills_far_low_goal_lateral_line_path, DRIVE_SPEED * .7, 19, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(5);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.set_max_speed(DRIVE_SPEED);
  chassis.wait_drive();
  aim_and_fire_far_goal();

  cata_intake.intake_stop();

  chassis.set_point_drive_pid(far_horizontal_roller + Vector2(-3, 4), DRIVE_SPEED, ez::FORWARD);
  chassis.wait_drive();
}

void skills2() {
  
  // default_constants();
  // exit_condition_defaults();
  // chassis.reset_position(Vector2(32, -11.5), Angle::from_deg(-88));
  // pros::delay(10); // Wait for odometry to use the reset orientation
  // chassis.set_orientation_turn_pid(Angle::from_deg(-90), TURN_SPEED); // Set heading PID
  // if(!cata_intake.cata_primed) { cata_intake.cata_prime(); }
  Vector2 expansion = Vector2(120, -120);

  roll(30, Angle::from_deg(-88.5), -.5, 50, 180);
  return;
  chassis.set_target_relative_swing_pid(ez::RIGHT_SWING, 140, SWING_SPEED, .14);
  chassis.wait_drive();
  chassis.set_drive_pid(7, DRIVE_SPEED);
  chassis.wait_until_distance_travelled(5.5);
  cata_intake.intake_velocity(INTK_IN);
  chassis.wait_drive();
  chassis.set_drive_pid(6, DRIVE_SPEED * .2);
  chassis.wait_drive();
  chassis.set_drive_pid(11, DRIVE_SPEED * .6);
  chassis.wait_drive();
  chassis.set_orientation_turn_pid(Angle::from_deg(-42), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(16, DRIVE_SPEED);
  chassis.wait_drive();
  cata_intake.intake_stop();
  roll(30, Angle::from_deg(1.5), -1, 60, 190);


  chassis.set_target_relative_swing_pid(ez::LEFT_SWING, -95.5, SWING_SPEED); 
  chassis.wait_drive(); // Swing into far goal
  cata_intake.intake_velocity(INTK_IN); // Continue intaking in case discs aren't settled
  chassis.set_target_relative_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-54.8, DRIVE_SPEED, true); 
  chassis.wait_drive(); // Drive to far goal
  chassis.set_target_relative_turn_pid(15, TURN_SPEED);
  chassis.wait_drive(); // Turn for first shot
  cata_intake.cata_shoot(); // First shot
  cata_intake.wait_cata_done_shot();
  /*** FIRST SHOT DONE ***/
  chassis.set_orientation_turn_pid(Angle::from_deg(182), TURN_SPEED); 
  chassis.wait_drive(); // Turn to slightly face low goal
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_drive_pid(29, DRIVE_SPEED * .8, true);
  chassis.wait_drive();
  chassis.set_orientation_turn_pid(Angle::from_deg(186), TURN_SPEED * .8);
  chassis.wait_drive(); // Face out a bit
  chassis.set_target_relative_swing_pid(ez::LEFT_SWING, 49, SWING_SPEED * .7);
  cata_intake.intake_time(400, INTK_IN); // Intake third disc, and probably extra one
  cata_intake.wait_intake(); 
  cata_intake.intake_velocity(-INTK_IN * .5); // Outtake extra disc
  chassis.wait_drive(); // Swing out to shot
  chassis.set_drive_pid(-2, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_target_relative_turn_pid(-8, TURN_SPEED);
  chassis.wait_drive(); // Position for second shot
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  /*** SECOND SHOT DONE ***/
  chasing_heading_constants();
  chassis.set_point_drive_pid(expansion, DRIVE_SPEED);
  default_constants();
  chassis.set_orientation_turn_pid(Angle::from_deg(-45), TURN_SPEED);
  chassis.wait_drive();
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



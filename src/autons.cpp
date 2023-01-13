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
    127; // This is 110/127 (around 87% of max speed).  We don't suggest making
         // this 127. If this is 127 and the robot tries to heading correct,
         // it's only correcting by making one side slower.  When this is 87%,
         // it's correcting by making one side faster and one side slower,
         // giving better heading correction.
const int LONG_INTAKE_DRIVE_SPEED = 90;
const int SHORT_INTAKE_DRIVE_SPEED = 70;
const int TURN_SPEED = 127;
const int SWING_SPEED = 110;
const int INTK_IN = 0.95*200*84/36;

///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier
// game objects, or with lifts up vs down. If the objects are light or the cog
// doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 2.7, 0.0028, 27, 10);
  chassis.set_pid_constants(&chassis.left_forward_drivePID, 0.245, 0.0018, 1.15, 300);
  chassis.set_pid_constants(&chassis.right_forward_drivePID, 0.245, 0.0018, 1.15, 300);
  chassis.set_pid_constants(&chassis.left_backward_drivePID, .265, 0, 1.38, 0);
  chassis.set_pid_constants(&chassis.right_backward_drivePID, .265, 0, 1.38, 0);
  chassis.set_pid_constants(&chassis.turnPID, 2.7, 0.0028, 27, 10);
  chassis.set_pid_constants(&chassis.swingPID, 6.8, 0, 50, 0);
  cata_intake.roller_set_pid_constants(5, 0.000, 14, 0);
  cata_intake.cata_set_pid_constants(5, 0.000, 14, 0);
}

void chasing_heading_constants() {
  //chassis.set_pid_constants(&chassis.headingPID, 4.2, 0, 45, 0);
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
  cata_intake.cata_set_exit_condition(100, 50, 500, 100, 1000, 1000);
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
  chassis.plan_orientation_heading_pid(target_orientation);
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
  //chassis.set_point_drive_pid(Vector2(24, -10), DRIVE_SPEED);
  //chassis.wait_drive();
  chassis.set_turn_pid(90);
  chassis.wait_drive();
  chassis.set_point_turn_pid(Vector2(7, 0), TURN_SPEED);
  chassis.wait_drive();
  //chassis.reset_position(Vector2(10.5, -29.5), Angle::from_deg(180));
  //pros::delay(10);
  //chassis.set_heading_relative_heading_pid(0);
  // chassis.set_turn_pid(14, TURN_SPEED);
  // chassis.wait_drive();
  // //pros::delay(000);
  // chassis.set_turn_pid(90, TURN_SPEED);
  // chassis.wait_drive();
  // chassis.set_turn_pid(0, TURN_SPEED);
  // chassis.wait_drive();
  
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
  chassis.plan_orientation_turn_pid(Angle::from_deg(0), TURN_SPEED);
  chassis.wait_drive();
  
}

void aim_and_fire_far_goal(Angle offset = Angle(), double runup = 0) {
  chassis.plan_point_turn_pid(far_goal, TURN_SPEED, Angle::from_deg(180) + offset);
  chassis.wait_drive();
  if(runup != 0) {
    chassis.set_drive_pid(runup, DRIVE_SPEED);
    chassis.wait_until_distance_travelled(runup + 7);
  }
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
  
}

void aim_and_fire_near_goal(Angle offset = Angle(), double runup = 0) {
  chassis.plan_point_turn_pid(near_goal, TURN_SPEED, Angle::from_deg(180) + offset);
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
  roll(30, Angle::from_deg(91.5), -.65, 70, 200);

  chassis.set_path_pid(skills_second_roller_path, DRIVE_SPEED, 8, ez::BACKWARD, 0);
  chassis.wait_until_absolute_points_passed(1);
  cata_intake.intake_velocity(INTK_IN * .7);
  chassis.set_point_path_orientation(ez::FORWARD);
  chassis.wait_until_distance_remaining(5);
  cata_intake.intake_stop();
  roll(30, Angle::from_deg(181.5), -.65, 70, 200);
  // start driving towards first shot
  chassis.set_path_pid(skills_first_shot_path, DRIVE_SPEED, 14, ez::BACKWARD);
  chassis.wait_until_absolute_points_passed(1);
  chassis.set_path_lookahead(12);
  cata_intake.intake_velocity(INTK_IN);
  chassis.wait_drive();
  aim_and_fire_far_goal();

  chassis.plan_orientation_turn_pid(Angle::from_deg(5), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_path_pid(skills_far_low_goal_horizontal_line_path, DRIVE_SPEED * .6, 14, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_max_speed(DRIVE_SPEED);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.wait_drive();
  aim_and_fire_far_goal(Angle::from_deg(-3)); // place a little to the left

  chassis.set_path_pid(skills_near_line_path, DRIVE_SPEED, 19, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  cata_intake.intake_velocity(INTK_IN * .8);
  chassis.wait_drive();
  aim_and_fire_far_goal(Angle::from_deg(-.75), -9.8);

  chassis.set_path_pid(skills_far_low_goal_lateral_line_path, DRIVE_SPEED * .6, 19, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(5);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.set_max_speed(DRIVE_SPEED);
  chassis.wait_drive();
  aim_and_fire_far_goal(Angle::from_deg(1)); // place a tad bit to the right

  cata_intake.intake_stop();

  chassis.set_point_drive_pid(far_horizontal_roller + Vector2(-3, 4), DRIVE_SPEED, ez::FORWARD);
  chassis.wait_drive();
}

void skills2() {

  roll(30, Angle::from_deg(-88.5), -.65, 70, 180);
  
  chassis.reset_position(Vector2(chassis.position.x, far_horizontal_roller.y), chassis.orientation);
  chassis.set_heading_relative_heading_pid(0);
  chassis.set_drive_pid(-10, DRIVE_SPEED);
  chassis.wait_until_distance_travelled(-2.5);
  chassis.set_heading_relative_turn_pid(200, TURN_SPEED);
  chassis.wait_until_heading_relative(180);
  chassis.set_path_pid(skills_far_corner_triple_stack_path, DRIVE_SPEED, 14, ez::FORWARD);
  chassis.set_point_path_orientation(ez::FORWARD);
  chassis.wait_until_distance_from_point(far_corner_triple_stack, 8.7);
  chassis.set_max_speed(DRIVE_SPEED * .16);
  chassis.set_path_lookahead(14);
  cata_intake.intake_velocity(INTK_IN);
  pros::delay(900);
  chassis.set_max_speed(DRIVE_SPEED * .6);
  chassis.wait_until_absolute_points_passed(4);
  chassis.set_max_speed(DRIVE_SPEED);
  chassis.wait_drive();
  cata_intake.intake_stop();
  
  roll(30, Angle::from_deg(1.5), -.65, 70, 190);

  chassis.reset_position(Vector2(far_lateral_roller.x + 1.0, chassis.position.y), chassis.orientation);
  chassis.set_heading_relative_heading_pid(0);
  // start driving towards first shot
  chassis.set_path_pid(skills_near_goal_first_shot_path, DRIVE_SPEED, 14, ez::BACKWARD);
  chassis.wait_until_absolute_points_passed(1);
  chassis.set_path_lookahead(12);
  cata_intake.intake_velocity(-INTK_IN);
  chassis.wait_drive();
  aim_and_fire_near_goal();
  
  cata_intake.intake_velocity(INTK_IN);

  chassis.plan_orientation_turn_pid(Angle::from_deg(-175), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_path_pid(skills_near_low_goal_horizontal_line_path, DRIVE_SPEED * .6, 14, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_max_speed(DRIVE_SPEED);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.wait_drive();
  aim_and_fire_near_goal(Angle::from_deg(1)); // place a little to the right
  
  chassis.set_path_pid(skills_far_line_path, DRIVE_SPEED, 15, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(3);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.wait_drive();
  aim_and_fire_near_goal(Angle::from_deg(2.5));

  chassis.set_path_pid(skills_near_low_goal_lateral_line_path, DRIVE_SPEED, 19, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(1);
  chassis.set_max_speed(DRIVE_SPEED * .6);
  chassis.wait_until_absolute_points_passed(5);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.set_max_speed(DRIVE_SPEED);
  chassis.wait_drive();

  aim_and_fire_near_goal();
  chassis.set_point_drive_pid(skills_near_expansion, DRIVE_SPEED);
  chassis.wait_until_distance_from_point(skills_near_expansion, 4);
  chassis.plan_orientation_turn_pid(Angle::from_deg(135), TURN_SPEED);
  chassis.wait_drive();
  
}

void turn_test() {
  chassis.plan_point_turn_pid(Vector2(), 80);
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



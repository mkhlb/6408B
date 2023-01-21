#include "autons.hpp"
#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"

#include "points.hpp"
#include "paths.hpp"
#include "pros/rtos.hpp"

const int DRIVE_SPEED = 127; 
const int ACCURATE_DRIVE_SPEED = 110;
const int LONG_INTAKE_DRIVE_SPEED = 90;
const int SHORT_INTAKE_DRIVE_SPEED = 70;
const int TURN_SPEED = 127;
const int SWING_SPEED = 110;
const int INTK_IN = 1.00*200*84/36;

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 2.50, 0.000, 22, 0);
  chassis.set_pid_constants(&chassis.left_forward_drivePID, 0.245, 0.0018, 1.15, 300);
  chassis.set_pid_constants(&chassis.right_forward_drivePID, 0.245, 0.0018, 1.15, 300);
  chassis.set_pid_constants(&chassis.left_backward_drivePID, .277, 0, 1.38, 0);
  chassis.set_pid_constants(&chassis.right_backward_drivePID, .277, 0, 1.38, 0);
  chassis.set_pid_constants(&chassis.turnPID, 3.4, 0.0028, 30, 18);
  chassis.set_pid_constants(&chassis.swingPID, 6.8, 0, 50, 0);
  cata_intake.roller_set_pid_constants(5, 0.000, 14, 0);
  cata_intake.cata_set_pid_constants(9, 0.001, 22, 50);
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
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
  chassis.headingPID.set_exit_condition(120, 3, 700, 7, 1000, 500);
  cata_intake.roller_set_exit_condition(50, 8, 90, 40, 800, 3000);
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

void roll(double max_dist, Angle target_orientation, double back_distance, double speed, double roll_amount, int wait_amount=200) // roll_amount is degrees
{
  chassis.plan_orientation_heading_pid(target_orientation);
  chassis.set_drive_pid(max_dist, speed); // drive forward 7 inches, or until meeting resistance
  pros::delay(wait_amount);
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

void drive_test() {
  default_constants();
  exit_condition_defaults();
  chassis.set_drive_pid(-3, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-6, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-39, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(48, DRIVE_SPEED);
  chassis.wait_drive();
}

void turn_test() {
  default_constants();
  exit_condition_defaults();
  chassis.set_turn_pid(8, TURN_SPEED);
  chassis.wait_drive();
  chassis.plan_orientation_turn_pid(Angle::from_deg(-45), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_heading_relative_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_target_relative_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  chassis.plan_orientation_turn_pid(Angle::from_deg(0), TURN_SPEED);
  chassis.wait_drive();
}

void point_turn_test() {
  chassis.plan_point_turn_pid(Vector2(0, -20), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(40, DRIVE_SPEED);
  chassis.wait_until_distance_travelled(24);
  chassis.set_point_turn_pid(Vector2(24, -24), TURN_SPEED, Angle::from_deg(180));
  chassis.wait_drive();
}

void point_drive_test() {
  chassis.set_point_drive_pid(Vector2(36, -24), DRIVE_SPEED, ez::BACKWARD);
  chassis.wait_drive();
  chassis.plan_orientation_turn_pid(Angle::from_deg(0), TURN_SPEED);
  chassis.wait_drive();
}

void odom_test() {
  default_constants();
  exit_condition_defaults();
  chassis.set_turn_pid(90, 80);
  chassis.wait_drive();
  chassis.set_point_turn_pid(Vector2(7, 0), 127);
  chassis.wait_drive();
  
}

void heading_test() {
  chassis.plan_orientation_heading_pid(Angle::from_deg(-70));
  chassis.set_drive_pid(60, DRIVE_SPEED);
  chassis.wait_drive();
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

  chassis.plan_orientation_turn_pid(Angle::from_deg(180), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_path_pid(DRIVE_SPEED, 18, ez::BACKWARD);
  chassis.wait_until_distance_travelled(10);
  chassis.set_max_speed(70);
  chassis.wait_until_absolute_points_passed(3);
  chassis.set_max_speed(110);
  chassis.wait_until_distance_remaining(24);
  chassis.set_max_speed(127);
  chassis.wait_drive();
  chassis.plan_orientation_turn_pid(Angle::from_deg(180), TURN_SPEED);
  chassis.wait_drive();
  
}

void aim_and_fire_far_goal(Angle offset = Angle(), double runup_target = 0, double runup_fire = -7) {
  chassis.set_point_turn_pid(far_goal, TURN_SPEED, Angle::from_deg(180) + offset);
  chassis.wait_drive();
  if(runup_target != 0) {
    chassis.set_drive_pid(runup_target, DRIVE_SPEED);
    chassis.wait_until_distance_travelled(runup_fire);
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


void print_pos() {
  master.print(0,0, "%f, %f", (float)chassis.position.x, (float)chassis.position.y);
}

void skills1() {
  std::uint32_t now = pros::millis();
  default_constants();
  exit_condition_defaults();
  cata_intake.cata_prime();
  chassis.reset_position(Vector2(skills_start.x, skills_start.y), Angle::from_deg(90.5));
  chassis.set_heading_relative_heading_pid(0);
  //pros::Task::delay_until(&now, 5000);
  //pros::Task::delay(0000);
  // chassis.set_drive_pid(-35.25, DRIVE_SPEED, false, true);
  // chassis.wait_drive();

  roll(30, Angle::from_deg(91.5), -.65, 70, 200);

  chassis.set_path_pid(skills_second_roller_path, DRIVE_SPEED, 8, ez::BACKWARD, 0);
  chassis.wait_until_absolute_points_passed(1);
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_point_path_orientation(ez::FORWARD);
  chassis.set_max_speed(ACCURATE_DRIVE_SPEED);
  chassis.wait_until_absolute_points_passed(2);
  cata_intake.intake_stop();
  roll(30, Angle::from_deg(181.5), -1.1, 70, 200);
  // start driving towards first shot
  chassis.set_path_pid(skills_first_shot_path, DRIVE_SPEED, 14, ez::BACKWARD);
  chassis.wait_until_absolute_points_passed(1);
  chassis.set_path_lookahead(20);
  cata_intake.intake_velocity(INTK_IN);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_max_speed(DRIVE_SPEED);
  //chassis.wait_until_distance_remaining(8);
  chassis.wait_until_distance_remaining(4);
  aim_and_fire_far_goal(Angle::from_deg(4));

  chassis.set_heading_relative_swing_pid(ez::LEFT_SWING, -90, TURN_SPEED, -.1);
  chassis.wait_until_heading_relative(-27);

  chassis.set_path_pid(skills_far_low_goal_horizontal_line_path, SHORT_INTAKE_DRIVE_SPEED, 14, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(1);
  cata_intake.intake_velocity(INTK_IN*.8);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_max_speed(ACCURATE_DRIVE_SPEED);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.wait_drive();
  chassis.wait_until_distance_remaining(2);
  aim_and_fire_far_goal(Angle::from_deg(0), -6, -1);

  chassis.set_path_pid(skills_near_line_path, LONG_INTAKE_DRIVE_SPEED, 19, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  cata_intake.intake_velocity(INTK_IN * .8);
  chassis.set_max_speed(SHORT_INTAKE_DRIVE_SPEED);
  chassis.wait_drive();
  aim_and_fire_far_goal(Angle::from_deg(0), -8.5, -4);

  chassis.set_path_pid(skills_far_low_goal_lateral_line_path, DRIVE_SPEED * .6, 19, ez::FORWARD);
  chassis.wait_drive();
  aim_and_fire_far_goal(Angle::from_deg(1.5)); // place a tad bit to the right

  cata_intake.intake_stop();

  chassis.set_point_drive_pid(far_horizontal_roller + Vector2(-3, 4), DRIVE_SPEED, ez::FORWARD);
  chassis.wait_until_distance_remaining(6);
}

void skills2() {

  roll(30, Angle::from_deg(-88.5), -.65, 70, 180, 600);
  
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


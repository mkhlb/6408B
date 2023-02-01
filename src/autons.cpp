#include "autons.hpp"
#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"

#include "paths.hpp"
#include "points.hpp"
#include "pros/rtos.hpp"


const int DRIVE_SPEED = 127;
const int ACCURATE_DRIVE_SPEED = 110;
const int LONG_INTAKE_DRIVE_SPEED = 90;
const int SHORT_INTAKE_DRIVE_SPEED = 70;
const int TURN_SPEED = 127;
const int SWING_SPEED = 110;
const int INTK_IN = 1.00 * 200 * 84 / 36;

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 2.50, 0.000, 22, 0);
  chassis.set_pid_constants(&chassis.left_forward_drivePID, 0.245, 0.0018, 1.15,
                            300);
  chassis.set_pid_constants(&chassis.right_forward_drivePID, 0.245, 0.0018,
                            1.15, 300);
  chassis.set_pid_constants(&chassis.left_backward_drivePID, .277, 0, 1.38, 0);
  chassis.set_pid_constants(&chassis.right_backward_drivePID, .277, 0, 1.38, 0);
  chassis.set_pid_constants(&chassis.turnPID, 3.4, 0.0028, 30, 18);
  chassis.set_pid_constants(&chassis.swingPID, 6.8, 0, 50, 0);
  cata_intake.roller_set_pid_constants(5, 0.000, 14, 0);
  cata_intake.cata_set_pid_constants(9, 0.001, 22, 50);
}

void chasing_heading_constants() {
  // chassis.set_pid_constants(&chassis.headingPID, 4.2, 0, 45, 0);
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

void exit_condition_early_drive() { // made to exit the drive way earlier, more
                                    // error so only use with position tracking
                                    // or if planning on recording error, made
                                    // when distance forward and backwards
                                    // doesn't matter too much
  chassis.set_exit_condition(chassis.drive_exit, 50, 6, 500, 10, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 50000, 7, 50000, 50000);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 50000, 7, 50000,
                             50000);
  chassis.set_exit_condition(chassis.drive_exit, 80, 25, 30000, 150, 50000,
                             50000);
}

void exit_condition_hit_wall() { // made to exit as soon as velocity is 0, for
                                 // detecting crashing into objects
  chassis.set_exit_condition(chassis.drive_exit, 100, 3, 500, 7, 30, 100);
}

// 6408B code
// __________________________________________________________________________________________________________________________________________________________________________________________________________

void roll(double max_dist, Angle target_orientation, double back_distance,
          double speed, double roll_amount,
          int wait_amount = 200) // roll_amount is degrees
{
  chassis.plan_orientation_heading_pid(target_orientation);
  chassis.set_drive_pid(
      max_dist, speed); // drive forward 7 inches, or until meeting resistance
  pros::delay(wait_amount);
  exit_condition_hit_wall(); // set exit conditions to conditions very sensitive
                             // to interference
  chassis.wait_drive();      // wait until drive exits
  exit_condition_defaults(); // reset exit conditions
  chassis.set_drive_pid(back_distance, speed);

  cata_intake.roller_pid_move(roll_amount, 127);
  cata_intake.wait_roller();
  cata_intake.roller_velocity(0);
}

void roll_time(double max_dist, double back_distance, double speed,
               double roll_time) { // roll time can be negative or positive
  exit_condition_hit_wall(); // set exit conditions to conditions very sensitive
                             // to interference
  chassis.set_drive_pid(
      max_dist, speed); // drive forward 7 inches, or until meeting resistance
  chassis.wait_drive(); // wait until drive exits
  exit_condition_defaults(); // reset exit conditions
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
  chassis.set_point_turn_pid(Vector2(24, -24), TURN_SPEED,
                             Angle::from_deg(180));
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
  chassis.add_point(Vector2(0, 0), 16);
  chassis.add_point(Vector2(24, 0), 16);
  chassis.add_point(Vector2(48, -18), 16);
  chassis.add_point(Vector2(48, -30), 16);
  chassis.add_point(Vector2(24, -48), 16);
  chassis.add_point(Vector2(0, -48), 16);
  // chassis.drive_to_points(DRIVE_SPEED);
  // chassis.drive_to_points(DRIVE_SPEED);

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

void aim_and_fire_far_goal(Angle offset = Angle(), double runup_target = 0,
                           double runup_fire = -7) {
  chassis.set_point_turn_pid(far_goal, TURN_SPEED,
                             Angle::from_deg(180) + offset);
  chassis.wait_drive();
  if (runup_target != 0) {
    chassis.set_drive_pid(runup_target, DRIVE_SPEED);
    chassis.wait_until_distance_travelled(runup_fire);
  }
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
}

void aim_and_fire(Vector2 goal, Angle offset = Angle(), double runup_target = 0,
                  double runup_fire = -7) {
  chassis.set_point_turn_pid(far_goal, TURN_SPEED,
                             Angle::from_deg(180) + offset);
  chassis.wait_drive();
  if (runup_target != 0) {
    chassis.set_drive_pid(runup_target, DRIVE_SPEED);
    chassis.wait_until_distance_travelled(runup_fire);
  }
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
}
void aim_and_fire_near_goal(Angle offset = Angle(), double runup = 0) {
  chassis.plan_point_turn_pid(near_goal, TURN_SPEED,
                              Angle::from_deg(180) + offset);
  chassis.wait_drive();
  if (runup != 0) {
    chassis.set_drive_pid(runup, DRIVE_SPEED);
    chassis.wait_until_distance_travelled(runup + 7);
  }
  cata_intake.cata_shoot();
  cata_intake.wait_cata_done_shot();
}

void intake_triple_stack(double distance = 34, double speed = 80) {
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_drive_pid(distance, speed, true);
  chassis.wait_drive();
}

void skills() {
  skills1();
  skills2();
}

void print_pos() {
  master.print(0, 0, "%f, %f", (float)chassis.position.x,
               (float)chassis.position.y);
}

void rotate_180(Vector2 field_size = Vector2(140.4, -140.4)) {
  chassis.reset_position(field_size - chassis.position,
                         chassis.orientation + Angle::from_deg(180));
}

void skills_shooting(Vector2 goal, std::list<PathPoint> long_line_path = skills_near_line_path) {
  chassis.set_path_pid(skills_first_shot_path, DRIVE_SPEED, 14, ez::BACKWARD);
  chassis.wait_until_absolute_points_passed(1);
  chassis.set_path_lookahead(20);
  cata_intake.intake_velocity(INTK_IN);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_max_speed(DRIVE_SPEED);
  // chassis.wait_until_distance_remaining(8);
  chassis.wait_until_distance_remaining(4);
  aim_and_fire_far_goal(Angle::from_deg(-.8));

  chassis.set_heading_relative_swing_pid(ez::LEFT_SWING, -90, TURN_SPEED, -.7);
  chassis.wait_until_heading_relative(-24);

  chassis.set_path_pid(skills_far_low_goal_horizontal_line_path,
                       SHORT_INTAKE_DRIVE_SPEED, 14, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(1);
  cata_intake.intake_velocity(INTK_IN * .8);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_max_speed(ACCURATE_DRIVE_SPEED);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.wait_drive();
  chassis.wait_until_distance_remaining(2);
  aim_and_fire_far_goal(Angle::from_deg(-1), -6.0, -1.0);

  chassis.set_path_pid(long_line_path, LONG_INTAKE_DRIVE_SPEED, 19,
                       ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  cata_intake.intake_velocity(INTK_IN * .8);
  chassis.set_max_speed(SHORT_INTAKE_DRIVE_SPEED);
  chassis.wait_drive();
  aim_and_fire_far_goal(Angle::from_deg(0), -8.0, -3.2);

  chassis.set_path_pid(skills_far_low_goal_lateral_line_path, DRIVE_SPEED * .6,
                       19, ez::FORWARD);
  chassis.wait_drive();
  aim_and_fire_far_goal(Angle::from_deg(0)); // place a tad bit to the right

  cata_intake.intake_stop();
  chassis.set_path_pid(skills_far_middle_triple_stack_path,
                       ACCURATE_DRIVE_SPEED, 19, ez::FORWARD);
  chassis.wait_until_distance_remaining(28.0);
  intake_triple_stack(28.5, 40);
  pros::delay(250);
  chassis.set_point_drive_pid(far_goal + Vector2(0, 2.5), 127);
  chassis.wait_until_axes_crossed(Vector2(46.64, -93.77) + Vector2(10, 10) +
                                      Vector2(14, 14),
                                  -1, -1); // low goal corner + some offset
  chassis.set_drive_pid(-14, 70);
  chassis.wait_until_distance_travelled(-10);
  cata_intake.cata_shoot();
  cata_intake.intake_stop();
  chassis.wait_until_distance_travelled(-11.5);
  chassis.set_heading_relative_turn_pid(0, TURN_SPEED);
  cata_intake.wait_cata_done_shot();
}

void skills1() {
  std::uint32_t now = pros::millis();
  default_constants();
  exit_condition_defaults();
  cata_intake.cata_prime();
  chassis.reset_position(Vector2(skills_start.x, skills_start.y),
                         Angle::from_deg(90.0));
  chassis.set_heading_relative_heading_pid(0);

  roll(30, Angle::from_deg(91.5), -.65, 70, 165);

  chassis.set_path_pid(skills_second_roller_path, DRIVE_SPEED, 8, ez::BACKWARD,
                       0);
  chassis.wait_until_absolute_points_passed(1);
  cata_intake.intake_velocity(INTK_IN);
  chassis.set_point_path_orientation(ez::FORWARD);
  chassis.set_max_speed(ACCURATE_DRIVE_SPEED);
  chassis.wait_until_absolute_points_passed(2);
  roll(30, Angle::from_deg(181.5), -1.1, 70, 165);
  // start driving towards first shot

  skills_shooting(far_goal);

  chassis.set_point_drive_pid(far_horizontal_roller + Vector2(0, 4.0),
                              DRIVE_SPEED, ez::FORWARD);
  chassis.wait_drive();
}

void skills2() {

  chassis.plan_orientation_turn_pid(Angle::from_deg(-90), TURN_SPEED);
  chassis.wait_until_orientation(Angle::from_deg(-45));

  roll(30, Angle::from_deg(-90), -.65, 70, 180, 600);

  chassis.reset_position(
      Vector2(chassis.position.x, far_horizontal_roller.y - 3.9),
      chassis.orientation);
  chassis.set_heading_relative_heading_pid(0);
  chassis.set_drive_pid(-11, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_point_turn_pid(far_corner_triple_stack, TURN_SPEED);
  chassis.wait_drive();
  intake_triple_stack(23, 40);
  chassis.set_heading_relative_swing_pid(ez::LEFT_SWING, -90, SWING_SPEED, -.3);
  chassis.wait_until_heading_relative(-45);
  chassis.set_path_pid(skills_far_corner_triple_stack_path, DRIVE_SPEED, 14,
                       ez::FORWARD, 3);
  chassis.set_max_speed(DRIVE_SPEED);
  chassis.wait_drive();
  cata_intake.intake_stop();

  chassis.plan_orientation_turn_pid(Angle::from_deg(0), TURN_SPEED);
  chassis.wait_drive();

  roll(30, Angle::from_deg(0), -.55, 70, 180);

  chassis.reset_position(
      Vector2(far_lateral_roller.x + 4.8, chassis.position.y),
      chassis.orientation);
  chassis.set_heading_relative_heading_pid(0);
  // start driving towards first shot

  rotate_180(Vector2(142, -142));
  skills_shooting(transposed_near_goal);
  rotate_180(Vector2(142, -142));

  chassis.set_point_drive_pid(skills_near_expansion, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.plan_orientation_turn_pid(Angle::from_deg(135), TURN_SPEED);
  chassis.wait_drive();

  // expand!
  expansion.move_velocity(200);
}

void prematch_near_first_shot() {
  chassis.set_path_pid(win_point_first_shot_path, DRIVE_SPEED, 19,
                       ez::BACKWARD);
  chassis.wait_until_absolute_points_passed(4);
  chassis.set_point_drive_pid(far_goal, DRIVE_SPEED, ez::BACKWARD);
  chassis.wait_until_distance_travelled(6);
  cata_intake.cata_shoot();
  chassis.set_drive_pid(-8, DRIVE_SPEED);
  cata_intake.wait_cata_done_shot();
}

void prematch_win_point() {
  default_constants();
  exit_condition_defaults();
  cata_intake.cata_prime();
  chassis.reset_position(skills_start, Angle::from_deg(90.0));
  chassis.set_heading_relative_heading_pid(0);

  roll(30, Angle::from_deg(90), -.65, 70, 80);

  prematch_near_first_shot();
  
  chassis.set_path_pid(win_point_second_shot_path, DRIVE_SPEED, 12, ez::FORWARD);
  chassis.wait_drive();
  aim_and_fire(far_goal, Angle(), -20, -5);
  chassis.set_path_pid(win_point_third_shot_path, DRIVE_SPEED, 12, ez::FORWARD);
  chassis.wait_drive();
  aim_and_fire(far_goal, Angle(), -26, -8);
  chassis.set_path_pid(win_point_roller_path, DRIVE_SPEED, 20, ez::FORWARD);
  roll(30, Angle::from_deg(0), -.55, 70, 80);
}

void prematch_near() {
  default_constants();
  exit_condition_defaults();
  cata_intake.cata_prime();
  chassis.reset_position(skills_start, Angle::from_deg(90.0));
  chassis.set_heading_relative_heading_pid(0);

  roll(30, Angle::from_deg(90), -.65, 70, 80);

  prematch_near_first_shot();

  chassis.set_path_pid(prematch_near_second_shot_path, DRIVE_SPEED, 18, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.wait_until_distance_remaining(4);

  aim_and_fire(far_goal, Angle(), -24, -9);
  
  chassis.set_path_pid(prematch_near_third_shot_path, DRIVE_SPEED, 18, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.wait_until_distance_remaining(4);

  aim_and_fire(far_goal, Angle(), -24, -9);

  chassis.set_path_pid(prematch_near_fourth_shot_path, DRIVE_SPEED, 18, ez::FORWARD);
  chassis.wait_until_absolute_points_passed(2);
  chassis.set_point_path_orientation(ez::BACKWARD);
  chassis.wait_until_distance_remaining(4);

  aim_and_fire(far_goal, Angle(), -24, -9);

  chassis.set_point_turn_pid(Vector2(59, -35), TURN_SPEED);
  chassis.wait_drive();
}

void prematch_far() {

}

void prematch_near_roller() {
  default_constants();
  exit_condition_defaults();
  cata_intake.cata_prime();
  chassis.reset_position(skills_start, Angle::from_deg(90.0));
  chassis.set_heading_relative_heading_pid(0);

  roll(30, Angle::from_deg(90), -.65, 70, 80);
}

void prematch_far_roller() {
  default_constants();
  exit_condition_defaults();
  cata_intake.cata_prime();
  chassis.reset_position(Vector2(132, -85), Angle::from_deg(0));
  chassis.set_heading_relative_heading_pid(0);

  chassis.set_path_pid(prematch_far_roller_path, ACCURATE_DRIVE_SPEED, 18, ez::FORWARD);
  chassis.wait_drive();

  roll(30, Angle::from_deg(0), -.65, 70, 80);
}

void null() { return; }
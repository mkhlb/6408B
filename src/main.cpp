#include "main.h"
#include "EZ-Template/datatypes.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/sdcard.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "points.hpp"
#include "paths.hpp"
#include "cata_intake.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

/////
/////
// For installation, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

pros::ADIDigitalOut boost(5, LOW);
pros::ADIDigitalOut expansion(2, LOW);

// Chassis constructor
Drive chassis(
    // Track width of chassis in inches
    4.625,
    4.625
    , 
    -4.0 // 0, overshoots : 1.4, undershoot
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {-10, 5, -20} // -13 front, -3 back
    
    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {14, -19, 3} // 18 front, 16 back


    // IMU Port
    ,
    11

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.25

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    360

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would
    // be 2.333. eg. if your drive is 36:60 where the 60t is powered, your RATIO
    // would be 0.6.
    ,
    1

    // Uncomment if using tracking wheels
    
    // Left Tracking Wheel Ports (negative port will reverse it!)
    ,-16 // 3 wire encoder
    // ,8 // Rotation sensor

    ,{0,0}

    ,2.75

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

mkhlib::CatapultIntakeController cata_intake(
    // Port of the motors
    {-4, 7},
    // Port of the limit switch
    1,
    
    5,
    // Ratio of roller revolutions / motor revolutions, motor revolution *
    // this ratio should = roller revolutions
    6.0 / 24.0,
    // Ratio of intake revolution / motor revolutions
    60.0 / 36.0,
    // gearset of catapult
    pros::E_MOTOR_GEARSET_06);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  //expansion.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cata_intake.cata_hold();
  // Print our branding over your terminal :D
  ez::print_ez_template();

  //cata_intake.cata_prime();
  pros::delay(
      500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls

  chassis.set_joystick_threshold(2);

  chassis.toggle_modify_curve_with_controller(
      false); // Enables modifying the controller curve with buttons on the
              // joysticks
  chassis.set_active_brake(.1, 70); // Sets the active brake kP. We recommend 0.1.
  chassis.set_acceleration(0, 0);
  chassis.set_deceleration(600, 1000);
  chassis.set_curve_default(3, 3.2); // Defaults for curve. If using tank, only
                                   // the first parameter is used.
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults();

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons(
    {
      Auton("Far side shooting", prematch_far),
      Auton("Point drive", point_drive_test),
      Auton("Win point", prematch_win_point),
      
      Auton("Far roller", prematch_far_roller),
      Auton("Near side shooting", prematch_near),
      Auton("Skills", skills),
      
    });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

  chassis.reset_pid_targets();               // Resets PID targets to 0.
  chassis.reset_gyro();                      // Reset gyro position to 0.
  chassis.reset_drive_sensor();              // Reset drive sensors to 0.
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps
                                             // autonomous consistency.


  //drive_test();
  //skills();
  //prematch_win_point();
  ez::as::auton_selector
      .call_selected_auton(); // Calls selected auton from autonomous
  //     selector.

  // skills();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void print_odom() {
  int counter_max = 5000;
  int counter = 50;
  double odom_error = 0;
  double last_degs = chassis.orientation.get_deg();
  printf("initializing printer \n");
  bool limitpresed = false;
  while (true) {
    //master.print(0,0, "%f, %f", chassis.position.x, chassis.position.y);
    if(cata_intake.limit.get_value() == 1 && !limitpresed) {
      limitpresed = true;
      //cata_intake.cata_reset_sensors();
    }
    else if(cata_intake.limit.get_value() != 1) {
      limitpresed = false;
    }
    master.print(0,0, "%f, %f", (float)chassis.orientation.get_deg(), (float)chassis.right_sensor());
    pros::delay(500);
  }
}

double aim_assist_coefficient(
    Vector2 goal_position, double goal_width, double min_coefficient,
    double angular_interp_start_error, double far_distance_interp_start,
    double far_distance_interp_end, double near_distance_interp_start,
    double near_distance_interp_end) { // interp start is furthest distance, end
                                       // is closest
  Vector2 robot_to_goal = goal_position - chassis.position;

  double absolute_error_to_center = abs(Angle::shortest_error(
      chassis.orientation, -robot_to_goal.get_angle_direction()));
  if (absolute_error_to_center >= 90) {
    return 1.0;
  }
  if (robot_to_goal.get_magnitude() == 0) {
    return 1.0;
  }
  // get the angle from the vector to the outside of the goal to the center of
  // it
  double absolute_outside_to_center =
      abs(atan(goal_width / 2 / robot_to_goal.get_magnitude()));

  // interpolation equation: out = ((in - in_at_max) * (out_at_min -
  // out_at_max)) / (in_at_min - in_at_max) + out_at_max

  // at absolute_outside_to_center = absolute_outside_to_center, =
  // min_cofficient, at outside_to_center = angular_interp_start_angle, = 1
  double angular_coefficient =
      1 + ((absolute_error_to_center - angular_interp_start_error) *
           (min_coefficient - 1)) /
              (absolute_outside_to_center - angular_interp_start_error);
  double clamped_angular_coefficient =
      max(min(angular_coefficient, 1.0), min_coefficient);
  // same 0 to 1 interpolation
  double distance = robot_to_goal.get_magnitude();

  double distance_coefficient = clamped_angular_coefficient;
  if (distance > far_distance_interp_end) {
    distance_coefficient = (distance - far_distance_interp_start) * (1.0 - clamped_angular_coefficient) / (far_distance_interp_end - far_distance_interp_start) + 1.0;
  } else if (distance < near_distance_interp_start) {
    distance_coefficient = (distance - near_distance_interp_start) * (clamped_angular_coefficient - 1.0) / (near_distance_interp_end - near_distance_interp_start) + clamped_angular_coefficient;
  }
  double clamped_distance_coefficient =
      max(clamped_angular_coefficient, min(1.0, distance_coefficient));

  return clamped_distance_coefficient;
}

void reset_for_driver() {
  chassis.set_mode(ez::DISABLE);
  chassis.reset_starts();
}

void opcontrol() {
  // This is preference to what you like to drive on.

  //autonomous();
  //pros::delay(6000);
  //return;
  boost.set_value(1);
  bool booststate = true;
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  cata_intake.cata_hold();

  double interpolator_end = 30.0;

  pros::Task odom_printer = pros::Task(print_odom);

  reset_for_driver();

  chassis.reset_position(skills_start, Angle::from_deg(90.0));
  //chassis.reset_position(Vector2(), Angle::from_deg(180));
  pros::delay(10);
  //ROBOT TO GOAL: 6.5, 94
  // field is about 142

  while (true) {

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      //chassis.plan_point_turn_pid(far_goal, 80, Angle::from_deg(180));
      chassis.set_point_drive_pid(far_goal + Vector2(0, 5), 127);
      chassis.wait_until_axes_crossed(Vector2(46.64, -93.77) + Vector2(10,10) + Vector2(14, 14), -1, -1); // low goal corner + some offset
      chassis.set_drive_pid(-14, 70);
      chassis.wait_until_distance_travelled(-7);
      reset_for_driver();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      booststate = !booststate;
      boost.set_value(booststate);
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      expansion.set_value(1);
    }
    else {
      //expansion.move_velocity(0);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      interpolator_end -= 5;
      master.print(0, 0, "%i", cata_intake.limit.get_value());
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      interpolator_end += 5;
      master.print(0, 0, "%f", interpolator_end);
    }

    
    double turn_coefficient = aim_assist_coefficient(
        Vector2(0, 0), 8.0, .5, 30.0, 42.0, 36.0, 20.0, 12.0);
    chassis.arcade_curvatherp_standard(ez::SPLIT, 2, interpolator_end,
                                         1); // curvatherp special split arcade

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {

      // cata_intake.cata_move_voltage(-60);
    
      cata_intake
          .cata_shoot(); // cata_intake shoot, moves cata_intake then shortly
                         // after waits until limit switch to stop it
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      cata_intake.cata_hold();
    }
    // else {
    //   cata_intake.cata_move_velocity(0);
    // }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      cata_intake.intake_velocity(.9 * 1000); // intake
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      cata_intake.intake_velocity(.5 * 1000); // outake
    } else {
      cata_intake.intake_stop(); // else to keep intake at 0
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}

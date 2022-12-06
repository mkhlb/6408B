#include "main.h"
#include "EZ-Template/sdcard.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

/////
/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    {16, 18}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {-13, -3}

    // IMU Port
    ,
    16

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.25

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    200

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would
    // be 2.333. eg. if your drive is 36:60 where the 60t is powered, your RATIO
    // would be 0.6.
    ,
    1.0

    // Uncomment if using tracking wheels
    /*
    // Left Tracking Wheel Ports (negative port will reverse it!)
    // ,{1, 2} // 3 wire encoder
    // ,8 // Rotation sensor

    // Right Tracking Wheel Ports (negative port will reverse it!)
    // ,{-3, -4} // 3 wire encoder
    // ,-9 // Rotation sensor
    */

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

mkhlib::CatapultIntakeController cata_intake(
    // Port of the catapult motor
    {-12, 19},
    // Port of the intake motor
    -20,
    // Port of the limit switch
    1,
    // Ratio of roller revolutions / motor revolutions, motor revolution *
    // this ratio should = roller revolutions
    1.0,
    // Ratio of roller revolution / motor revolutions
    1.0,
    //gearset of catapult
    pros::E_MOTOR_GEARSET_18,
    //gearset of intake
    pros::E_MOTOR_GEARSET_18);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();

  cata_intake.cata_hold();
  pros::delay(
      500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls

  chassis.toggle_modify_curve_with_controller(
      false); // Enables modifying the controller curve with buttons on the
              // joysticks
  chassis.set_active_brake(0.1); // Sets the active brake kP. We recommend 0.1.
  cata_intake.intake_roller_set_active_brake(.25);
  chassis.set_curve_default(
      1, 2); // Defaults for curve. If using tank, only the first parameter
             // is used.
  default_constants(); // Set the drive to your own constants from autons.cpp!

  exit_condition_defaults();

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Full win point", roll_test)});

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

  cata_intake.cata_hold();

  ez::as::auton_selector
      .call_selected_auton(); // Calls selected auton from autonomous selector.

  // auto selection

  // roll_test();
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

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  cata_intake.cata_hold();

  double interpolator_end = 30.0;

  while (true) {

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      interpolator_end -= 5;
      master.print(0, 0, "%f", interpolator_end);
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      interpolator_end += 5;
      master.print(0, 0, "%f", interpolator_end);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      chassis.arcade_standard(ez::SPLIT);
    } else {
      chassis.arcade_curvatherp_standard(
          ez::SPLIT, 2, interpolator_end); // curvatherp special split arcade
      // chassis.arcade_curvatherp_standard(ez::SPLIT);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {

      cata_intake
          .cata_shoot(); // cata_intake shoot, moves cata_intake then shortly
                         // after waits until limit switch to stop it
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      cata_intake.intake_velocity(0.9 * 200); // intake
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      cata_intake.intake_velocity(-100); // outake
    } else {
      cata_intake.intake_stop(); // else to keep intake at 0
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}

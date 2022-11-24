#include "main.h"
#include "EZ-Template/sdcard.hpp"
#include "EZ-Template/util.hpp"
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
    {-19, -10, 9}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {1, 17, -2}

    // IMU Port
    ,
    16

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    4.125

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would
    // be 2.333. eg. if your drive is 36:60 where the 60t is powered, your RATIO
    // would be 0.6.
    ,
    7.0 / 3.0

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
  11, 
  // Port of the intake motor
  12, 
  // Port of the limit switch
  1, 
  // Ratio of roller revolutions / intake revolutions, intake revolution * this ratio should = roller revolutions
  1.0f/3.0f, 
  // Wether or not to reverse the catapult motor, the cata should shoot in reverse
  true
);

pros::ADIDigitalOut poonamic(8);

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
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.


  // Configure your chassis controls

  chassis.toggle_modify_curve_with_controller(
      false); // Enables modifying the controller curve with buttons on the
              // joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(
      0.1, 0.1); // Defaults for curve. If using tank, only the first parameter
                 // is used. (Comment this line out if you have an SD card!)
  default_constants(); // Set the drive to your own constants from autons.cpp!

  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!
  
  
  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Full win point", winpoint_full),
    Auton("half win point (only first roller and shot)", winpoint_half),
    Auton("single shot pre match for bad side", farside_roller),
    Auton("Auto skills", auto_skills),
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

  cata_intake.cata_hold();

  //ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.

  // auto selection

  //roll_test();
  auto_skills();
  //winpoint_half();
  //winpoint_full();
  //farside_roller();
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

  bool upPressed = false;

  int expansion_timer = 60000 - 10 * 1000; // 1:00 - 0:10 in ms

  //int expansion_timer = 105000 - 10 * 1000; // 1:45 - 0:10 in ms

  cata_intake.cata_hold();

  while (true) {

    // chassis.tank(); // Tank control
    chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      // autonomous();
      ez::as::auton_selector.print_selected_auton();
    }

    // . . .
    // Put more user control code here!
    // . . .

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {

      cata_intake.cata_clear();
      upPressed = true;
    } else if (upPressed) {
      upPressed = false;
      cata_intake.cata_hold();
    }

    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      
    //   cata_intake.cata_prime(); //moves cata_intake until contact with the limit switch
    // }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      
      cata_intake.cata_shoot(); //cata_intake shoot, moves cata_intake then shortly after waits until limit switch to stop it
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      cata_intake.intake_velocity(0.6*600); //intake
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      cata_intake.intake_velocity(-300); //outake
    } 
    else {
      cata_intake.intake_velocity(0); //else to keep intake at 0
    }
    
    // only move intake if cata_intake is primed
    // if (cata_intake.cata_primed) 
    // {
    //   if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    //     cata_intake.intake_velocity(0.6*600); //intake
    //   }
    //   else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    //     cata_intake.intake_velocity(-300); //outake
    //   } 
    //   else {
    //     cata_intake.intake_velocity(0); //else to keep intake at 0
    //   }
    // }
    // else if(!master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    //   cata_intake.intake_velocity(0);
    // }
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) { //backup intaking button

    //   cata_intake.intake_velocity(0.6*600); //intake

    // } 

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
        (expansion_timer <= 0 || !pros::competition::is_connected())) {

      poonamic.set_value(true); // Laucnhes Pneumatics expansion
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME

    expansion_timer -= ez::util::DELAY_TIME;
  }
}

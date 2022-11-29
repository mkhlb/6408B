#include "cata_intake.hpp"

#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>

using namespace mkhlib;

CatapultIntakeController::CatapultIntakeController(int cata_port, int intake_port, int limit_switch_port, double motor_to_roller_ratio, double motor_to_intake_ratio,  pros::motor_gearset_e cata_gearset, pros::motor_gearset_e intake_gearset)
    : limit(limit_switch_port),
      intake(abs(intake_port), intake_gearset, ez::util::is_reversed(intake_port), pros::E_MOTOR_ENCODER_DEGREES),
      MOTOR_TO_ROLLER(motor_to_roller_ratio),
      MOTOR_TO_INTAKE(motor_to_intake_ratio),
      cata_loop([this] { this->master_cata_task(); }),
      intake_loop([this] { this->master_intake_task(); }) {

  
  // Populate the list of cata motors
  pros::Motor temp(abs(cata_port), cata_gearset, ez::util::is_reversed(cata_port));
  temp.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);

  cata_motors.push_back(temp);


  // Default states
  cata_state = e_cata_state::HOLD; 
  roller_state = e_roller_state::IDLE;


  max_speed = 600; // Max speed internally is in terms of the intake motor, but when set you pass it in terms of the roller

  roller_interfered = false;

  // Default internal timer for e_roller_state::TIME_MOVE
  _roller_timer = 0;

  // Default PID for e_roller_state::PID_MOVE
  roller_pid = PID(.5, 0, 5, 0);
}

CatapultIntakeController::CatapultIntakeController(std::vector<int> cata_ports, int intake_port, int limit_switch_port, double motor_to_roller_ratio, double motor_to_intake_ratio, pros::motor_gearset_e cata_gearset, pros::motor_gearset_e intake_gearset)
    : limit(limit_switch_port),
      intake(abs(intake_port), intake_gearset, ez::util::is_reversed(intake_port), pros::E_MOTOR_ENCODER_DEGREES),
      MOTOR_TO_ROLLER(motor_to_roller_ratio),
      MOTOR_TO_INTAKE(motor_to_intake_ratio),
      cata_loop([this] { this->master_cata_task(); }),
      intake_loop([this] { this->master_intake_task(); }) {
  
  // Populate list of cata motors
  for (auto i : cata_ports) {
    pros::Motor temp(abs(i), cata_gearset, ez::util::is_reversed(i));
    temp.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);

    cata_motors.push_back(temp);
  }

  // Default states
  cata_state = e_cata_state::HOLD; 
  roller_state = e_roller_state::IDLE;

  max_speed = 600; // Max speed internally is in terms of the intake motor, but when set you pass it in terms of the roller

  roller_interfered = false;

  // Default internal timer for e_roller_state::TIME_MOVE
  _roller_timer = 0;

  // Default PID for e_roller_state::PID_MOVE
  roller_pid = PID(.5, 0, 5, 0);
  
}

void CatapultIntakeController::master_cata_task() {

  // Master control loop: state machine
  while (true) {  
  
    if( cata_state == e_cata_state::HOLD) // Lock motors during hold state.
    {
      for (auto i : cata_motors) {
        i.move_velocity(0);
      }
    }
    else if( cata_state == e_cata_state::CLEAR) // Slowly move cata up
    {
      for (auto i : cata_motors) {
        i.move_voltage(.9 * 12000);
      }
    }
    else if( cata_state == e_cata_state::PRIME) // Run the prime function each tick while in the prime state
    {
      cata_prime_task();
    }
    else if( cata_state == e_cata_state::SHOOT) // Run the shoot function each tick while in the shoot state
    {
      cata_shoot_task();
    }

    cata_primed = cata_state == e_cata_state::HOLD; // Update the public cata_primed variable - used for intake safety

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}

void CatapultIntakeController::cata_prime_task() { // Gets called every tick cata is in PRIME state
  
  for (auto i : cata_motors) { // Iterate all motors
    i.move_voltage(-12000); // Move at max speed into prime position
  }
    
  if(limit.get_value()) // Stop when limit switch is pressed
  {
    pros::delay(40); // Short delay to get more square contact with switch
    cata_state = e_cata_state::HOLD;
  }
  
}

void CatapultIntakeController::cata_shoot_task() {

  for (auto i : cata_motors) { // Iterate all motors
    i.move_voltage(-12000); // Move until limit switch is no longer pressed
  }
  if(!limit.get_value())
  {
    pros::delay(100); // Wait short while before priming in case limit switch is pressed again on the way up

    cata_state = e_cata_state::PRIME;
  }
}

void CatapultIntakeController::wait_cata_idle() { // Waits until cata state is HOLD
  while (cata_state != e_cata_state::HOLD) {
    pros::delay(util::DELAY_TIME);
  }
}

void CatapultIntakeController::wait_done_shot() { // Waits untill cata is done SHOOT
  while (cata_state == e_cata_state::SHOOT) {
    pros::delay(util::DELAY_TIME);
  }
}

// Set cata state:

void CatapultIntakeController::cata_hold() { cata_state = e_cata_state::HOLD; }

void CatapultIntakeController::cata_prime() { cata_state = e_cata_state::PRIME; }

void CatapultIntakeController::cata_shoot() { cata_state = e_cata_state::SHOOT; }

void CatapultIntakeController::cata_clear() { cata_state = e_cata_state::CLEAR; }

void CatapultIntakeController::master_intake_task() {
  // Intake/roller state controller
  while(true)
  {
    if(!cata_primed) {
      intake.move_velocity(0); // Catapult safety, pause all other actions while catapult is up
    }
    else if(roller_state == e_roller_state::PID_MOVE) {
      roller_pid_task(); // Run the roller PID function
    }
    else if(roller_state == e_roller_state::TIME_MOVE) {
      roller_intake_spin_time_task(); // Run the spin time function
    }
    else if (roller_state == e_roller_state::IDLE) {
      intake.move_velocity(_intake_velocity); // Move roller at set speed (or stop it) if it's safe to do so
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}

void CatapultIntakeController::roller_pid_task() {
  roller_pid.compute(intake.get_position());

  //clip output
  double out = ez::util::clip_num(roller_pid.output, max_speed, -max_speed);

  intake.move_velocity(out);

  // check for exit

  exit_output exit = roller_pid.exit_condition(intake);
  if(exit != ez::RUNNING) { //PID HAS EXITED
    roller_interfered = exit == ez::mA_EXIT || exit == ez::VELOCITY_EXIT;

    intake_stop();
  }
}

void CatapultIntakeController::roller_intake_spin_time_task() {
  if(_roller_timer > 0) {
    _roller_timer -= ez::util::DELAY_TIME; // Decrement roller timer by delta time
  }
  else {
    intake_stop(); // Call intake_stop to transition state and stop the intake
  }
  
}

void CatapultIntakeController::intake_velocity(double velocity) {
  roller_state = e_roller_state::IDLE;
  _intake_velocity = velocity / MOTOR_TO_INTAKE; // Convert between given intake RPM and the motor RPM
}

void CatapultIntakeController::intake_stop() {
  roller_state = e_roller_state::IDLE;
  _intake_velocity = 0;
}

void CatapultIntakeController::roller_velocity(double velocity) {
  roller_state = e_roller_state::IDLE;
  _intake_velocity = velocity / MOTOR_TO_ROLLER; // Convert between given roller RPM and the motor RPM
}

void CatapultIntakeController::roller_stop() {
  roller_state = e_roller_state::IDLE;
  _intake_velocity = 0;
}

void CatapultIntakeController::roller_time(int time, double velocity) {
  intake.move_velocity(velocity / MOTOR_TO_ROLLER); // Convert between given roller RPM and the motor RPM
  _roller_timer = time;
  roller_state = e_roller_state::TIME_MOVE;
}

void CatapultIntakeController::intake_time( int time, double velocity) {
  intake.move_velocity(velocity / MOTOR_TO_INTAKE); // Convert between given intake RPM and the motor RPM
  _roller_timer = time;
  roller_state = e_roller_state::TIME_MOVE;
}

void CatapultIntakeController::roller_pid_move(double target, int speed) { //Target should be in degrees of the roller, speed should again be in speed of the roller
  
  max_speed = speed / MOTOR_TO_ROLLER; // Convert between given roller RPM and the motor RPM

  _roller_start = intake.get_position();

  double target_encoder = _roller_start + (target / MOTOR_TO_ROLLER); // Convert between given roller target and the motor target

  roller_pid.set_target(target_encoder);

  roller_state = e_roller_state::PID_MOVE; // Set state to start PID logic
}

void CatapultIntakeController::roller_set_exit_condition(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout){
  roller_pid.set_exit_condition(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
}

void CatapultIntakeController::wait_roller() {
  while(roller_state != e_roller_state::IDLE) {
    pros::delay(util::DELAY_TIME);
  }
}

void CatapultIntakeController::wait_intake() {
  wait_roller();
}
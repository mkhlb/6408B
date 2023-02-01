#include "cata_intake.hpp"

#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
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
  pros::Motor temp(abs(cata_port), cata_gearset, ez::util::is_reversed(cata_port), pros::E_MOTOR_ENCODER_DEGREES);
  temp.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);

  cata_motors.push_back(temp);


  // Default states
  cata_state = e_cata_state::HOLD; 
  roller_state = e_roller_state::IDLE;


  intake_max_speed = 600; // Max speed internally is in terms of the intake motor, but when set you pass it in terms of the roller

  roller_interfered = false;

  cata_primed = false;

  // Default internal timer for e_roller_state::TIME_MOVE
  _roller_timer = 0;

  // Default PID for e_roller_state::PID_MOVE
  roller_pid = PID(.5, 0, 5, 0);
  cata_pid = PID(.5, 0, 5, 0);

  switch (cata_gearset) {
    case pros::E_MOTOR_GEARSET_06: _cata_max_velocity = 600; break;
    case pros::E_MOTOR_GEARSET_18: _cata_max_velocity = 200; break;
    case pros::E_MOTOR_GEARSET_36: _cata_max_velocity = 100; break;
    case pros::E_MOTOR_GEARSET_INVALID: _cata_max_velocity = 0; break;
  }
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
    pros::Motor temp(abs(i), cata_gearset, ez::util::is_reversed(i), pros::E_MOTOR_ENCODER_DEGREES);
    temp.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);

    cata_motors.push_back(temp);
  }

  // Default states
  cata_state = e_cata_state::HOLD; 
  roller_state = e_roller_state::IDLE;

  intake_max_speed = 600; // Max speed internally is in terms of the intake motor, but when set you pass it in terms of the roller

  roller_interfered = false;

  cata_primed = false;

  // Default internal timer for e_roller_state::TIME_MOVE
  _roller_timer = 0;

  intake.tare_position();

  // Default PID for e_roller_state::PID_MOVE
  roller_pid = PID(.5, 0, 5, 0);
  cata_pid = PID(.5, 0, 5, 0);

  switch (cata_gearset) {
    case pros::E_MOTOR_GEARSET_06: _cata_max_velocity = 600; break;
    case pros::E_MOTOR_GEARSET_18: _cata_max_velocity = 200; break;
    case pros::E_MOTOR_GEARSET_36: _cata_max_velocity = 100; break;
    case pros::E_MOTOR_GEARSET_INVALID: _cata_max_velocity = 0; break;
  }
}

void CatapultIntakeController::master_cata_task() {

  // Master control loop: state machine
  while (true) {  
    if( cata_state == e_cata_state::HOLD) // Lock motors during hold state.
    {
      // cata_pid.compute(cata_motors.front().get_position());
      // cata_move_voltage(util::clip_num(cata_pid.output, 60, -60));
      // if(cata_pid.exit_condition() != ez::RUNNING) {
      //   cata_move_velocity(0);
      // }
    }
    else if( cata_state == e_cata_state::CLEAR) // Slowly move cata up
    {
      cata_move_velocity(.9 * _cata_max_velocity);
    }
    else if( cata_state == e_cata_state::PRIME) // Run the prime function each tick while in the prime state
    {
      cata_prime_task();
    }
    else if( cata_state == e_cata_state::SHOOT) // Run the shoot function each tick while in the shoot state
    {
      cata_shoot_task();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

void CatapultIntakeController::cata_move_voltage(double voltage) {
  for (auto i : cata_motors) { // Iterate all motors
    i.move_voltage(voltage / 127 * 12000); // Move at max speed into prime position
  }
}

void CatapultIntakeController::cata_move_velocity(double velocity) {
  for (auto i : cata_motors) { // Iterate all motors
    i.move_velocity(velocity); // Move at max speed into prime position
  }

}

void CatapultIntakeController::cata_move_relative(double position, double velocity) {
  for (auto i : cata_motors) { // Iterate all motors
    i.move_relative(position, velocity); // Move at max speed into prime position
  }
  //cata_pid.set_target(cata_motors.front().get_position() + position);
}

void CatapultIntakeController::cata_reset_sensors() {
  for (auto i : cata_motors) { // Iterate all motors
    i.tare_position(); // Move at max speed into prime position
  }
}

void CatapultIntakeController::cata_prime_task() { // Gets called every tick cata is in PRIME state
  
  cata_move_velocity(-_cata_max_velocity * .9);
    
  if(limit.get_value() == 1) // Stop when limit switch is pressed
  {
    _cata_extra_error = -20;
    cata_reset_sensors();
    cata_move_relative(_cata_extra_error / 36.0 * 84.0, _cata_max_velocity * .6);
    
    cata_primed = true;
    cata_state = e_cata_state::HOLD;
  }
  
}

void CatapultIntakeController::cata_shoot_task() {

  cata_move_velocity(-_cata_max_velocity * .8);

  if(!limit.get_value())
  {
    pros::delay(400); // Wait short while before priming in case limit switch is pressed again on the way up

    cata_state = e_cata_state::PRIME;
  }
}

void CatapultIntakeController::wait_cata_idle() { // Waits until cata state is HOLD
  while (cata_state != e_cata_state::HOLD) {
    pros::delay(util::DELAY_TIME);
  }
}

void CatapultIntakeController::wait_cata_done_shot() { // Waits untill cata is done SHOOT
  while (cata_state == e_cata_state::SHOOT) {
    pros::delay(util::DELAY_TIME);
  }
}

// Set cata state:

void CatapultIntakeController::cata_hold() { 
  cata_state = e_cata_state::HOLD; 
  cata_move_velocity(0);

}

void CatapultIntakeController::cata_prime() { if(!cata_primed) { cata_state = e_cata_state::PRIME; } }

void CatapultIntakeController::cata_shoot() { cata_state = e_cata_state::SHOOT; }

void CatapultIntakeController::cata_clear() { cata_state = e_cata_state::CLEAR; }

void CatapultIntakeController::master_intake_task() {
  // Intake/roller state controller
  while(true)
  {
    if(!_intake_safety_bypass && !cata_primed) {
      intake.move_velocity(0); // Catapult safety, pause all other actions while catapult is up
    }
    else if(roller_state == e_roller_state::PID_MOVE) {
      roller_pid_task(); // Run the roller PID function
    }
    else if(roller_state == e_roller_state::TIME_MOVE) {
      roller_intake_spin_time_task(); // Run the spin time function
    }
    else if (roller_state == e_roller_state::IDLE) {
      if(_intake_roller_active_brake_kp != 0 && _intake_velocity != 0){
        intake.move_velocity(_intake_velocity); // Move roller at set speed (or stop it) if it's safe to do so
      }
      else { // intake_stop and roller_stop properly set the desired position to 0, important to use them!
        intake.move_voltage(/*0 */- intake.get_position() * _intake_roller_active_brake_kp * (12000.0 / 127.0));
      }
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}

void CatapultIntakeController::intake_roller_set_active_brake(double kp) { _intake_roller_active_brake_kp = kp; }

void CatapultIntakeController::roller_pid_task() {
  roller_pid.compute(intake.get_position());

  //clip output
  double out = ez::util::clip_num(roller_pid.output, intake_max_speed, -intake_max_speed);

  intake.move_voltage(out * 12000.0 / 127.0);

  // check for exit

  exit_output exit = roller_pid.exit_condition(intake);
  if(exit != ez::RUNNING) { //PID HAS EXITED!
    roller_interfered = exit == ez::mA_EXIT || exit == ez::VELOCITY_EXIT;

    
    intake_stop();
  }
}

void CatapultIntakeController::roller_intake_spin_time_task() {
  if(_roller_timer > 0) {
    intake.move_velocity(_intake_velocity);
    _roller_timer -= ez::util::DELAY_TIME; // Decrement roller timer by delta time
  }
  else {
    intake_stop(); // Call intake_stop to transition state and stop the intake
  }
  
}

void CatapultIntakeController::intake_velocity(double velocity) {
  roller_state = e_roller_state::IDLE;
  _intake_velocity = velocity / MOTOR_TO_INTAKE; // Convert between given intake RPM and the motor RPM
  _intake_safety_bypass = false;
}

void CatapultIntakeController::intake_stop() {
  if(_intake_velocity != 0 || roller_state != IDLE) {intake.tare_position(); }
  roller_state = e_roller_state::IDLE;
  _intake_velocity = 0;
}

void CatapultIntakeController::roller_velocity(double velocity) {
  roller_state = e_roller_state::IDLE;
  _intake_velocity = velocity / MOTOR_TO_ROLLER; // Convert between given roller RPM and the motor RPM
  _intake_safety_bypass = true;
}

void CatapultIntakeController::roller_stop() {
  roller_state = e_roller_state::IDLE;
  if(_intake_velocity != 0) {intake.tare_position(); }
  _intake_velocity = 0;
}

void CatapultIntakeController::roller_time(int time, double velocity) {
  _intake_velocity = velocity / MOTOR_TO_ROLLER; // Convert between given roller RPM and the motor RPM
  _roller_timer = time;
  roller_state = e_roller_state::TIME_MOVE;
  _intake_safety_bypass = true;
}

void CatapultIntakeController::intake_time( int time, double velocity) {
  _intake_velocity = velocity / MOTOR_TO_INTAKE; // Convert between given intake RPM and the motor RPM
  _roller_timer = time;
  roller_state = e_roller_state::TIME_MOVE;
  _intake_safety_bypass = false;
}

void CatapultIntakeController::roller_pid_move(double target, int speed) { //Target should be in degrees of the roller, speed should again be in speed of the roller
  
  intake_max_speed = speed; // Convert between given roller RPM and the motor RPM

  _roller_start = intake.get_position();

  double target_encoder = _roller_start + (target / MOTOR_TO_ROLLER); // Convert between given roller target and the motor target

  roller_pid.set_target(target_encoder);

  roller_state = e_roller_state::PID_MOVE; // Set state to start PID logic

  _intake_safety_bypass = true;
}

void CatapultIntakeController::roller_set_exit_condition(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout){
  roller_pid.set_exit_condition(p_small_exit_time, p_small_error / MOTOR_TO_ROLLER, p_big_exit_time, p_big_error / MOTOR_TO_ROLLER, p_velocity_exit_time, p_mA_timeout);
}

void CatapultIntakeController::roller_set_pid_constants(double kp, double ki, double kd, double start_i) {
  roller_pid.set_constants(kp, ki, kd, start_i / MOTOR_TO_ROLLER);
}

void CatapultIntakeController::cata_set_pid_constants(double kp, double ki, double kd, double start_i) {
  cata_pid.set_constants(kp, ki, kd, start_i);
}

void CatapultIntakeController::cata_set_exit_condition(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout) {
  cata_pid.set_exit_condition(p_small_exit_time, p_small_error, p_big_exit_time, p_big_error, p_velocity_exit_time, p_mA_timeout);
}

void CatapultIntakeController::wait_roller() {
  while(roller_state != e_roller_state::IDLE) {
    pros::delay(util::DELAY_TIME);
  }
}

void CatapultIntakeController::wait_intake() {
  wait_roller();
}

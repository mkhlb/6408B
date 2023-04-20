#include "cata_intake.hpp"

#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <iostream>

using namespace mkhlib;



CatapultIntakeController::CatapultIntakeController(std::vector<int> ports, int limit_switch_port, int boost_port, double motor_to_roller_ratio, double motor_to_intake_ratio, pros::motor_gearset_e motors_gearset)
    : limit(limit_switch_port),
      boost(boost_port, LOW),
      MOTOR_TO_ROLLER(motor_to_roller_ratio),
      MOTOR_TO_INTAKE(motor_to_intake_ratio),
      logic_loop([this] { this->master_task(); }) {
  
  // Populate list of cata motors
  for (auto i : ports) {
    pros::Motor temp(abs(i), motors_gearset, ez::util::is_reversed(i), pros::E_MOTOR_ENCODER_DEGREES);
    temp.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);

    motors.push_back(temp);

    std::cout << i;
  }

  // Default states
  state = e_state::HOLD;

  cata_primed = false;

  // Default internal timer for e_roller_state::TIME_MOVE
  _roller_timer = 0;

  reset_sensors();

  switch (motors_gearset) {
    case pros::E_MOTOR_GEARSET_06: motors_max_speed = 600; break;
    case pros::E_MOTOR_GEARSET_18: motors_max_speed = 200; break;
    case pros::E_MOTOR_GEARSET_36: motors_max_speed = 100; break;
    case pros::E_MOTOR_GEARSET_INVALID: motors_max_speed = 0; break;
  }
}

void CatapultIntakeController::master_task() {

  // Master control loop: state machine
  while (true) {  
    if( state == e_state::HOLD) // Lock motors during hold state.
    {
      intake_move_velocity(_intake_velocity);
    }
    else if( state == e_state::PRIME) // Run the prime function each tick while in the prime state
    {
      cata_prime_task();
    }
    else if( state == e_state::SHOOT) // Run the shoot function each tick while in the shoot state
    {
      cata_shoot_task();
    }
    else if(state == e_state::CATA_DEGREES) {
      cata_spin_degrees_task();
    }
    else if(state == e_state::INTAKE_DEGREES) {
      roller_intake_spin_degrees_task(); // Run the roller PID function
    }
    else if(state == e_state::INTAKE_TIME) {
      roller_intake_spin_time_task(); // Run the spin time function
    }

    pros::delay(10);
  }
}

void CatapultIntakeController::cata_move_voltage(double voltage) {
  for (auto i : motors) { // Iterate all motors
    i.move_voltage(abs(voltage) / 127 * -12000); // Move at max speed into prime position
  }
}

void CatapultIntakeController::cata_move_velocity(double velocity) {
  for (auto i : motors) { // Iterate all motors
    i.move_velocity(-abs(velocity)); // Move at max speed into prime position
  }

}

void CatapultIntakeController::reset_sensors() {
  for (auto i : motors) { // Iterate all motors
    i.tare_position(); // Move at max speed into prime position
  }
}

void CatapultIntakeController::cata_prime_task() { // Gets called every tick cata is in PRIME state
  
  cata_move_velocity(motors_max_speed * .85);
    
  if(limit.get_value() == 1) // Tunable extra movement for the caterpult
  {
    //pros::delay(150);
    cata_primed = true;
    
    for (auto i : motors) {
      i.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      i.move_velocity(0);
    }
    pros::delay(100);
    for (auto i : motors) {
      i.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    
    
    state = e_state::HOLD;
    
  }
  
}

void CatapultIntakeController::cata_shoot_task() { // move catapult for constant time to fire
  
  int total_delay = 150;
  if(_boost_time >= total_delay) {
    boost.set_value(1);
    cata_move_velocity(-motors_max_speed * .85);
  }
  else {
    cata_move_velocity(-motors_max_speed * .8);
  }
  cata_primed = false;
  
  if(limit.get_value() != 1) {
    cata_move_voltage(-127 * .2);
    
    if (_boost_time < total_delay) {
      pros::delay(total_delay - _boost_time);
      boost.set_value(1);
      pros::delay(_boost_time);
    }
    else {
      pros::delay(total_delay);
    }
    
    
    // for (auto i : motors) {
    //   i.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // }
    boost.set_value(0);
    // cata_move_velocity(0);
    // pros::delay(90);
    // for (auto i : motors) {
    //   i.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // }
    cata_prime();
  }
  

  
  
}

void CatapultIntakeController::cata_spin_degrees_task() {
  if(motors.front().get_position() < _roller_target) {
    cata_hold();
  }
  else {
    cata_move_velocity(_cata_velocity);
  }
}

void CatapultIntakeController::wait_cata_idle() { // Waits until cata state is HOLD
  while (state == e_state::PRIME || state == e_state::SHOOT) {
    pros::delay(util::DELAY_TIME);
  }
}

void CatapultIntakeController::wait_cata_done_shot() { // Waits untill cata is done SHOOT
  while (state == e_state::SHOOT) {
    pros::delay(util::DELAY_TIME);
  }
}

// Set cata state:

void CatapultIntakeController::cata_hold() { 
  state = e_state::HOLD; 
}

void CatapultIntakeController::cata_prime() { 
  if(!cata_primed) { state = e_state::PRIME; }
}

void CatapultIntakeController::cata_shoot(int boost_time) { 
  _boost_time = boost_time;
  state = e_state::SHOOT;
}

void CatapultIntakeController::roller_intake_spin_degrees_task() {
  if(motors.front().get_position() > _roller_target) {
    _intake_velocity = 0; // Call intake_stop to transition state and stop the intake
    state = HOLD;
  }
  else {
    intake_move_velocity(_intake_velocity);
  }
}

void CatapultIntakeController::roller_intake_spin_time_task() {
  if(_roller_timer > 0) {
    intake_move_velocity(_intake_velocity);
    _roller_timer -= 10; // Decrement roller timer by delta time
  }
  else {
    _intake_velocity = 0; // Call intake_stop to transition state and stop the intake
    state = HOLD;
  }
  
}

void CatapultIntakeController::intake_velocity(double velocity) {
  _intake_velocity = velocity / MOTOR_TO_INTAKE; // Convert between given intake RPM and the motor RPM
  _intake_safety_bypass = false;
}

void CatapultIntakeController::intake_stop() {
  _intake_velocity = 0;
}



void CatapultIntakeController::intake_move_velocity(double velocity) {
  for (auto i: motors) {
    i.move_velocity(abs(velocity));
  }
}

void CatapultIntakeController::intake_move_voltage(double velocity) {
  for (auto i: motors) {
    i.move_voltage(abs(velocity));
  }
}

void CatapultIntakeController::roller_velocity(double velocity) {
  _intake_velocity = velocity / MOTOR_TO_ROLLER; // Convert between given roller RPM and the motor RPM
  _intake_safety_bypass = true;
}

void CatapultIntakeController::roller_stop() {
  _intake_velocity = 0;
}

void CatapultIntakeController::roller_time(int time, double velocity) {
  _intake_velocity = velocity / MOTOR_TO_ROLLER; // Convert between given roller RPM and the motor RPM
  _roller_timer = time;
  state = e_state::INTAKE_TIME;
  _intake_safety_bypass = true;
}

void CatapultIntakeController::intake_time(int time, double velocity) {
  _intake_velocity = velocity / MOTOR_TO_INTAKE; // Convert between given intake RPM and the motor RPM
  _roller_timer = time;
  state = e_state::INTAKE_TIME;
  _intake_safety_bypass = false;
}

void CatapultIntakeController::roller_degrees(double target, int speed) {
  reset_sensors();
  _intake_velocity = speed;
  _roller_target = target;
  state = e_state::INTAKE_DEGREES;
  _intake_safety_bypass = true;
}

void CatapultIntakeController::cata_degrees(double target, int speed) {
  reset_sensors();
  _cata_velocity = speed;
  _roller_target = target;
  state = e_state::CATA_DEGREES;

}

void CatapultIntakeController::wait_roller() {
  while(state == e_state::INTAKE_TIME || state == e_state::INTAKE_DEGREES) {
    pros::delay(util::DELAY_TIME);
  }
}

void CatapultIntakeController::wait_intake() {
  wait_roller();
}

void CatapultIntakeController::set_boost(bool value) {
  boost.set_value(int(value));
}

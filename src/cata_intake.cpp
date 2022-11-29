#include "cata_intake.hpp"

#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>

using namespace mkhlib;

CatapultIntakeController::CatapultIntakeController(int cata_port, int intake_port, int limit_switch_port, double intake_to_roller_ratio)
    : limit(limit_switch_port),
      intake(abs(intake_port), pros::motor_gearset_e::E_MOTOR_GEARSET_06, ez::util::is_reversed(intake_port), pros::E_MOTOR_ENCODER_DEGREES),
      INTAKE_TO_ROLLER(intake_to_roller_ratio),
      cata_loop([this] { this->master_cata_task(); }),
      intake_loop([this] { this->master_intake_task(); }) {

  pros::Motor temp(abs(cata_port), pros::motor_gearset_e::E_MOTOR_GEARSET_36, ez::util::is_reversed(cata_port));
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

CatapultIntakeController::CatapultIntakeController(std::vector<int> cata_ports, int intake_port, int limit_switch_port, double intake_to_roller_ratio)
    : limit(limit_switch_port),
      intake(abs(intake_port), pros::motor_gearset_e::E_MOTOR_GEARSET_06, ez::util::is_reversed(intake_port), pros::E_MOTOR_ENCODER_DEGREES),
      INTAKE_TO_ROLLER(intake_to_roller_ratio),
      cata_loop([this] { this->master_cata_task(); }),
      intake_loop([this] { this->master_intake_task(); }) {
  
  for (auto i : cata_ports) {
    pros::Motor temp(abs(i), pros::motor_gearset_e::E_MOTOR_GEARSET_36, ez::util::is_reversed(i));
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
  while (true) {

    if( cata_state == e_cata_state::HOLD)
    {
      for (auto i : cata_motors) {
        i.move_velocity(0);
      }
    }
    else if( cata_state == e_cata_state::CLEAR)
    {
      for (auto i : cata_motors) {
        i.move_velocity(90);
      }
    }
    else if( cata_state == e_cata_state::PRIME)
    {
      cata_prime_task();
    }
    else if( cata_state == e_cata_state::SHOOT)
    {
      cata_shoot_task();
    }

    cata_primed = cata_state == e_cata_state::HOLD;

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}

void CatapultIntakeController::cata_prime_task() {
  
  for (auto i : cata_motors) {
    i.move_velocity(-100);
  }
    
  if(limit.get_value())
  {
    pros::delay(40);
    cata_state = e_cata_state::HOLD;
  }
  
}

void CatapultIntakeController::cata_shoot_task() {

  for (auto i : cata_motors) {
    i.move_velocity(-100);
  }
  if(!limit.get_value())
  {
    pros::delay(100);

    cata_state = e_cata_state::PRIME;
  }
}

void CatapultIntakeController::wait_cata_idle() {
  while (cata_state != e_cata_state::HOLD) {
    pros::delay(util::DELAY_TIME);
  }
}

void CatapultIntakeController::wait_done_shot() {
  while (cata_state == e_cata_state::SHOOT) {
    pros::delay(util::DELAY_TIME);
  }
}

void CatapultIntakeController::cata_hold() { cata_state = e_cata_state::HOLD; }

void CatapultIntakeController::cata_prime() { cata_state = e_cata_state::PRIME; }

void CatapultIntakeController::cata_shoot() { cata_state = e_cata_state::SHOOT; }

void CatapultIntakeController::cata_clear() { cata_state = e_cata_state::CLEAR; }

void CatapultIntakeController::master_intake_task() {
  while(true)
  {
    
    if(!cata_primed) {
      intake.move_velocity(0);
    }
    else if(roller_state == e_roller_state::PID_MOVE) {
      roller_pid_task();
    }
    else if(roller_state == e_roller_state::TIME_MOVE) {
      roller_intake_spin_time_task();
    }
    else if (roller_state == e_roller_state::IDLE) {
      intake.move_velocity(_intake_velocity);
    }
    

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}

void CatapultIntakeController::roller_pid_task() {
  roller_pid.compute(intake.get_position());

  //clip output
  double out = ez::util::clip_num(roller_pid.output, max_speed, -max_speed);

  //clip speed when robot within startI
  // if(roller_pid.constants.ki != 0 && (fabs(roller_pid.get_target()) > roller_pid.constants.start_i && fabs(roller_pid.error) < roller_pid.constants.start_i)) {

  // }

  intake.move_velocity(out);

  // check for exit

  exit_output exit = roller_pid.exit_condition(intake);
  if(exit != ez::RUNNING) { //PID HAS EXITED
    roller_interfered = exit == ez::mA_EXIT || exit == ez::VELOCITY_EXIT;

    intake_velocity(0);
  }
}

void CatapultIntakeController::roller_intake_spin_time_task() {
  if(_roller_timer > 0) {
    _roller_timer -= ez::util::DELAY_TIME;
  }
  else {
    intake_velocity(0);
  }
  
}

void CatapultIntakeController::intake_velocity(double velocity) {
  roller_state = e_roller_state::IDLE;
  _intake_velocity = velocity;
}

void CatapultIntakeController::roller_velocity(double velocity) {
  roller_state = e_roller_state::IDLE;
  _intake_velocity = velocity / INTAKE_TO_ROLLER;
}

void CatapultIntakeController::roller_time(int time, double velocity) {
  intake_time(velocity / INTAKE_TO_ROLLER, time);
}

void CatapultIntakeController::intake_time( int time, double velocity) {
  intake.move_velocity(velocity);
  _roller_timer = time;
  roller_state = e_roller_state::TIME_MOVE;
}

void CatapultIntakeController::roller_pid_move(double target, int speed) { //Target should be in degrees of the roller, speed should again be in speed of the roller
  max_speed = ez::util::clip_num(speed, 600 * INTAKE_TO_ROLLER, -600 * INTAKE_TO_ROLLER) / INTAKE_TO_ROLLER;

  _roller_start = intake.get_position();

  double target_encoder = _roller_start + (target / INTAKE_TO_ROLLER);

  roller_pid.set_target(target_encoder);

  roller_state = e_roller_state::PID_MOVE;
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
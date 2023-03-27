#pragma once

#include "EZ-Template/PID.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <vector>

namespace mkhlib {
class CatapultIntakeController {
public:
  enum e_state { HOLD = 0, PRIME = 1, SHOOT = 2, INTAKE_TIME = 3, INTAKE_DEGREES = 4, CATA_DEGREES = 5};

  
  std::vector<pros::Motor> motors;

  pros::ADIDigitalIn limit;

  pros::ADIDigitalOut boost;

  double MOTOR_TO_ROLLER;

  double MOTOR_TO_INTAKE;

  /**
   * Creates a catapult controller.
   *
   * \param motor_port
   *        Port of the catapult motor.
   * \param limit_switch_port
   *        Port of the limit switch.
   * \param reverse
   *        Wether or not to reverse the catapult motor. Position should be up,
   * true by default.
   */
  CatapultIntakeController(std::vector<int> ports, int limit_switch_port, int boost_port, double motor_to_roller_ratio, double motor_to_intake_ratio, pros::motor_gearset_e motors_gearset=pros::motor_gearset_e::E_MOTOR_GEARSET_36);

  /**
   * @brief Sets the catapult state to HOLD.
   *
   * Sets the underlying control loop to hold the catapult motor. Never
   * transitions to another state.
   */
  void cata_hold();
  /**
   * @brief Sets the catapult state to PRIME.
   *
   * Sets the underlying control loop to go down until the limit switch is
   * contacted, upon which it transitions to the HOLD state
   */
  void cata_prime();
  /**
   * @brief Sets the catapult state to SHOOT.
   *
   * Sets the underlying control loop to go down until the limit switch is no
   * longer contacted, meaning that the catapult has fired, upon which it
   * transitions to the PRIME state.
   */
  void cata_shoot(bool boost=false);
  /**
   * Waits until the control loop enters the HOLD state.
   *
   *
   */
  void wait_cata_idle();
  /**
   * Waits until the control loop is not in the SHOOT state.
   *
   *
   */
  void wait_cata_done_shot();

  void cata_move_velocity(double velocity);
  void cata_move_voltage(double voltage);

  void reset_sensors();

  e_state state;

  bool cata_primed;


  //INTAKE AND ROLLER CONTROL STUFF

  double motors_max_speed;

  /**
   * @brief Sets the intake's velocity
   *
   * \param velocity the RPM velocity to set the intake to
   */
  void intake_velocity(double velocity);
  /**
   * @brief Stops the intake - sets its velocity to 0
   *
   */
  void intake_stop();
  /**
   * @brief Sets the intake to spin for set time
   *
   * \param time the time in milliseconds to spin the intake for
   *
   * \param velocity the RPM velocity to set the intake to
   */
  void intake_time(int time, double velocity);
  /**
   * @brief Sets the roller's velocity
   *
   * \param velocity the roller RPM velocity to set the roller to
   */
  void roller_velocity(double velocity);
  /**
   * @brief Stops the roller - sets its velocity to 0
   *
   */
  void roller_stop();
  /**
   * @brief Sets the roller to spin for set time
   *
   * \param time the time in milliseconds to spin the roller for
   *
   * \param velocity the RPM velocity (in roller revolutions) to set the roller to
   */
  void roller_time(int time, double velocity);
  /**
   * @brief Sets the roller PID's target
   *
   * \param target the target in relative degrees to set the PID to
   *
   * \param speed the maximum RPM speed (in roller revolutions) that the roller should get
   */
  void roller_degrees(double target, int speed);
  
  void cata_degrees(double target, int speed);

  /**
   * Waits until the roller is not moving
   *
   *
   */
  void wait_roller();
  /**
   * Waits until the intake is not moving
   *
   *
   */
  void wait_intake();

private:
  pros::Task logic_loop;

  void master_task();

  void cata_prime_task();
  void cata_shoot_task();

  void cata_spin_degrees_task();

  void set_boost(bool value);

  void roller_intake_spin_degrees_task();
  void roller_intake_spin_time_task();

  void intake_move_velocity(double velocity);
  void intake_move_voltage(double voltage);

  //INTAKE ROLLER STUFF
  double _roller_target;

  int _roller_timer;

  double _intake_velocity = 0;

  double _cata_velocity = 0;

  bool _intake_safety_bypass = false;

};
}; // namespace mkhlib
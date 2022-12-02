#pragma once

#include "EZ-Template/PID.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

namespace mkhlib {
class CatapultIntakeController {
public:
  enum e_cata_state { HOLD = 0, PRIME = 1, SHOOT = 2, CLEAR = 3 };

  
  std::vector<pros::Motor> cata_motors;

  pros::Motor intake;

  pros::ADIDigitalIn limit;

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
  CatapultIntakeController(int cata_port, int intake_port, int limit_switch_port, double motor_to_roller_ratio, double motor_to_intake_ratio, pros::motor_gearset_e cata_gearset=pros::motor_gearset_e::E_MOTOR_GEARSET_36, pros::motor_gearset_e intake_gearset=pros::motor_gearset_e::E_MOTOR_GEARSET_06);
  
  CatapultIntakeController(std::vector<int> cata_ports, int intake_port, int limit_switch_port, double motor_to_roller_ratio, double motor_to_intake_ratio, pros::motor_gearset_e cata_gearset=pros::motor_gearset_e::E_MOTOR_GEARSET_36, pros::motor_gearset_e intake_gearset=pros::motor_gearset_e::E_MOTOR_GEARSET_06);

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
   * @brief Sets the catapult state to CLEAR.
   *
   * Sets the underlying control loop to go up at a slow speed to clear jams.
   * Never transitions to another state, ONLY INTENDED FOR DRIVER CONTROL AS YOU
   * HAVE TO STOP IT YOURSELF.
   */
  void cata_clear();
  /**
   * @brief Sets the catapult state to SHOOT.
   *
   * Sets the underlying control loop to go down until the limit switch is no
   * longer contacted, meaning that the catapult has fired, upon which it
   * transitions to the PRIME state.
   */
  void cata_shoot();
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

  e_cata_state cata_state;

  bool cata_primed;


  //INTAKE AND ROLLER CONTROL STUFF

  enum e_roller_state {IDLE = 0, PID_MOVE = 1, TIME_MOVE = 2};

  PID roller_pid;

  double max_speed;

  bool roller_interfered;

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
  void roller_pid_move(double target, int speed);
  /**
   * @brief Sets the roller PID's exit conditions
   *
   * \param p_small_exit_time
   *        Sets small_exit_time.  Timer for to exit within small_error.
   * \param p_small_error
   *        Sets smalL_error. Timer will start when error is within this.
   * \param p_big_exit_time
   *        Sets big_exit_time.  Timer for to exit within big_error.
   * \param p_big_error
   *        Sets big_error. Timer will start when error is within this.
   * \param p_velocity_exit_time
   *        Sets velocity_exit_time.  Timer will start when velocity is 0.
   */
  void roller_set_exit_condition(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout);

  /**
   * Waits until the roller is IDLE
   *
   *
   */
  void wait_roller();
  /**
   * Waits until the intake is IDLE
   *
   *
   */
  void wait_intake();

  e_roller_state roller_state;

private:
  pros::Task cata_loop;

  pros::Task intake_loop;

  void master_cata_task();
  void master_intake_task();

  void cata_move_velocity(double velocity);

  void cata_prime_task();
  void cata_shoot_task();

  void roller_pid_task();
  void roller_intake_spin_time_task();

  //INTAKE ROLLER STUFF
  double _roller_start;

  int _roller_timer;

  double _intake_velocity;

  double _cata_max_velocity;
};
}; // namespace mkhlib
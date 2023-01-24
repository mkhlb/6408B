from math import pi

class Analyzer:

  INCHES_TO_METERS = 0.0254
  METERS_TO_INCHES = 39.3701

  def __init__(self, motor_torque: float, wheel_diameter: float, drive_motors_count: int, mass: float, drivetrain_speed: float): #motor force is 1.1 Nm but use .5 to be safe
    self.wheel_diameter = wheel_diameter * self.INCHES_TO_METERS
    self.mass = mass
    
    motor_force = motor_torque / (self.wheel_diameter / 2)
    one_side_force = motor_force * drive_motors_count / 2
    total_force = motor_force * drive_motors_count

    self.one_side_acceleration = one_side_force / mass
    self.acceleration = total_force / mass

    revolutions_per_second = drivetrain_speed / 60
    circumference = self.wheel_diameter * pi

    self.velocity = circumference * revolutions_per_second

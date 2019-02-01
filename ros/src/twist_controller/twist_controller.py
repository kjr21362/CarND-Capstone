import math
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                   wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        
        # PID parameters
        self.kp_t = 0.1
        self.ki_t = 0.1
        self.kd_t = 0.1
        self.kp_s = 0.1
        self.ki_s = 0.1
        self.kd_s = 0.1
        self.pid_t = PID(self.kp_t, self.ki_t, self.kd_t)
        self.pid_s = PID(self.kp_s, self.ki_s, self.kd_s)
        
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0, self.max_lat_accel, self.max_steer_angle)

    def control(self, linear_vel, angular_vel, current_linear_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if dbw_enabled:
            total_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY
            throttle = self.pid_t.step(linear_vel - current_linear_vel, 1.0/50)
            brake = 0
            if throttle < 0:
                brake = total_mass * math.fabs(throttle) * self.wheel_radius
            steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_linear_vel)
            return throttle, brake, steer
        else:
            self.pid_t.reset()
            self.pid_s.reset()
        return 1., 0., 0.

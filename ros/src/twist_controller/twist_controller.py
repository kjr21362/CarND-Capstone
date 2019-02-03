import math
from pid import PID
from yaw_controller import YawController
import rospy

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
        self.kp_t = 3.0
        self.ki_t = 0.
        self.kd_t = 0.
        min_throttle = 0.0
        max_throttle = 0.2
        self.pid_t = PID(self.kp_t, self.ki_t, self.kd_t, min_throttle, max_throttle)
        self.last_time = rospy.get_time()
        
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0, self.max_lat_accel, self.max_steer_angle)

    def control(self, linear_vel, angular_vel, current_linear_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if dbw_enabled:
            total_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY
            vel_error = linear_vel - current_linear_vel
            current_time = rospy.get_time()
            sample_time = current_time - self.last_time
            self.last_time = current_time
            throttle = self.pid_t.step(vel_error, sample_time)
            brake = 0
            
            # Stop at red light
            if linear_vel == 0 and current_linear_vel < 0.1:
                throttle = 0
                brake = 400
            elif vel_error < 0:
                decel = max(self.decel_limit, vel_error)
                brake = total_mass * math.fabs(decel) * self.wheel_radius
            steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_linear_vel)
            return throttle, brake, steer
        else:
            self.pid_t.reset()
            return 0., 0., 0.

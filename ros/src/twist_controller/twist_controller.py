from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement 
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius

        self.steering_controller  = YawController(wheel_base, steer_ratio, ONE_MPH, max_lat_accel, max_steer_angle)
        self.throttle_controller = PID(kp=0.3, ki=0.1, kd=0, mn=0, mx=0.2)

        self.low_pass_filter = LowPassFilter(0.5, 0.02)
       
        self.last_timestamp = rospy.get_time()
        

    def control(self, linear_velocity,angular_velocity,current_velocity,dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        current_velocity = self.low_pass_filter.filt(current_velocity)
        steering = self.steering_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        cte = linear_velocity - current_velocity
        
        current_timestamp = rospy.get_time()
        sample_time = current_timestamp - self.last_timestamp
        self.last_timestamp = current_timestamp
        throttle = self.throttle_controller.step(cte, sample_time)
        brake = 0
        
        if linear_velocity == 0 and current_velocity < 0.1:
            throttle = 0
            brake = 400
        elif throttle < 0.1 and cte < 0:
            throttle = 0
            decel = max(cte, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
        return throttle, brake, steering


        

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg  import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport  # Drive-by-wire messages
from geometry_msgs.msg import TwistStamped

from yaw_controller import YawController  # yaw_controller.py
from pid import PID                       # pid.py
from lowpass import LowPassFilter         # lowpass.py

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

MIN_SPEED = 0.1

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                       accel_limit, wheel_radius, wheel_base, steer_ratio, 
                       max_lat_accel, max_steer_angle):
        
        # TODO: Implement
        self.max_steer_angle = max_steer_angle
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        
        self.last_time = rospy.get_time()        
        self.vehicle_mass   = vehicle_mass
        self.fuel_capacity  = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit    = decel_limit
        self.accel_limit    = accel_limit
        self.wheel_radius   = wheel_radius
        
        min_speed = 0

        # Controllers
        kp = rospy.get_param('Kp', -0.2)
        ki = rospy.get_param('Ki', -0.002)
        kd = rospy.get_param('Kd', -24)
        mn = 0.      # minimum throttle value
        mx = 0.2     # maximum throttle value
        self.throttle_controller = PID( kp, ki, mn, mx)

        tau = 0.5    # 1/(2pi*tau) = cutoff frequency
        ts  = 0.02   # sample time
        self.lpf = LowPassFilter(tau, ts)
        
        self.yaw_controller = YawController( wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)    

    """
    :@ linear_vel : target linear velocity
    :@ angular_vel: target angular velocity
    """
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # calculate CTE
        cte = current_vel - linear_vel
        self.last_vel = cte

        if (dbw_enabled and self.last_time):
            current_time    = rospy.get_time()
            sample_time     = current_time - self.last_time
            self.last_time  = current_time

            # throttle control
            throttle = self.throttle_controller.step( cte, sample_time)
            throttle = max(-1., min(throttle, 1.))
            
            throttle = max(0.0, throttle)
            brake = max(0.0, (-throttle + self.brake_deadband))
                    
            # steering control
            steering  = self.yaw_controller.get_steering( linear_vel, angular_vel, current_vel)
            steering = max(-self.max_steer_angle, min(steering, self.max_steer_angle))
        else:
            self.last_time = rospy.get_time()
            throttle = 1.
            brake = 0.
            steering = 0.

        steering = self.lpf.filt(steering)
        
        # try to limit the throttle for big steer angle
        if (steering > 0.5):
            throttle = 0.1*throttle
        elif (steering > 0.3):
            throttle = 0.3*throttle
        

        '''
        # If target linear velocity = 0, then go very slow        
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0.
            brake    = 400   # N*m - to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2
        
        # If throttle is really small and velocity error < 0 (i.e. we're going faster than we want to be)
        elif throttle < 0.1 and vel_error < 0.:
            throttle = 0.
            decel    = max( vel_error, self.decel_limit)  # a negative number
            brake    = abs(decel) * self.vehicle_mass * self.wheel_radius   # Torque N*m
            
        '''        
 
        return throttle, brake, steering

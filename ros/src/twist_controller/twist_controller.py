import rospy
from pid import PID
from lowpass import LowPassFilter
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
        self.wheel_base	= wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.last_time = None

        # setup the yaw controller for steering
        self.yaw_controller = YawController(wheel_base = wheel_base, steer_ratio = steer_ratio, min_speed = ONE_MPH,
                                            max_lat_accel = max_lat_accel, max_steer_angle = max_steer_angle)

        # setup the PID controller parameters for the accelerator/brake
        self.accel_pid = PID(kp=1., ki=0., kd=0., mn=decel_limit, mx=accel_limit)
        
        # setup the low pass filter for steering
        self.steer_lpf = LowPassFilter(tau = 3., ts = 1.)
        
        # setup the low pass filter for acceleration
        self.accel_lpf = LowPassFilter(tau = 3., ts = 1.)
        #pass

    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        # only proceed if dbw is enabled; else error may accumulate in control loops
        if not dbw_enabled:
            return 0., 0., 0.

        # don't send commands if this is first time through loop
        if self.last_time is None:
            self.last_time = rospy.get_time()
            return 0.0, 0.0, 0.0
          
        # calculate delta time for the PID controller
        delta_time = rospy.get_time() - self.last_time

        rospy.loginfo("proposed_linear_velocity %f, current_linear_velocity %f", proposed_linear_velocity, current_linear_velocity)
        rospy.loginfo("proposed_angular_velocity %f", proposed_angular_velocity)
        velocity_error = proposed_linear_velocity - current_linear_velocity

        steering = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
        steer_cmd = self.steer_lpf.filt(steering)

        acceleration = self.accel_pid.step(velocity_error, delta_time)
        acceleration = self.accel_lpf.filt(acceleration)

        if acceleration > 0.0:
            throttle_cmd = acceleration
            brake_cmd = 0.0
        else:
            throttle_cmd = 0.0
            brake_cmd = abs(acceleration)

            # only brake if commanded braking higher than deadband
            if brake_cmd < self.brake_deadband:
                brake_cmd = 0.0

            # translate percentage of braking to braking force desired
            brake_cmd = brake_cmd * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        #return 1., 0., 0.
        return throttle_cmd, brake_cmd, steer_cmd

   def reset(self):
	self.accel_pid.reset()

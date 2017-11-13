from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.steer_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)	
	self.steer_filter = LowPassFilter(1, 2)	
	self.speed_controller = PID(1, 0, 0, mn=-1., mx=1.)
	self.speed_filter = LowPassFilter(1, 1)
	pass	
	
	
    def control(self, target_velocity, target_angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	steer_angle = self.steer_controller.get_steering(current_velocity, target_angular_velocity, current_velocity)
	steer_angle = self.steer_filter.filt(steer_angle)

	speed_err = target_velocity-current_velocity
	accel = self.speed_controller.step(speed_err, 0.02)
	accel = self.speed_filter.filt(accel)
	if accel >= 0:
	    gas_pedal = accel
	    brake_pedal = 0.
	else:
	    gas_pedal = 0
	    brake_pedal = -3000*accel
	#rospy.logwarn('speed_err: %s, val_i: %s, accel: %s, gas_pedal: %s, brake_pedal: %s', speed_err, self.speed_controller.int_val, accel, gas_pedal, brake_pedal)
        return gas_pedal, brake_pedal, steer_angle

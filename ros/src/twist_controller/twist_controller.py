import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
brake_torque_to_keep_zero_speed = 450


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
                 decel_limit, accel_limit, wheel_radius, wheel_base, 
                 steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        
        rospy.loginfo("Controller init started")
        # Assign Inputs to class attributes
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limt = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        
        # Create Lowpassfilter attribute for speed values
        ts = 0.02 # 50 Hz sample time
        tau = 0.5 # cut-off frequency
        
        self.vel_lowpassfiltered = LowPassFilter(tau, ts)
        
        # Throttle Controller Initialization:
        throttle_kp = 0.3
        throttle_ki = 0.1
        throttle_kd = 0.
        throttle_min = 0.0
        throttle_max = 0.4
        
        # Throttle Controller
        self.throttle_controller = PID(throttle_kp, throttle_ki, throttle_kd, throttle_min, throttle_max)
        
        # Steering Angle Controller
        #self.steering_controller = PID(10, 10, 10)
        
        # Yaw Controller:
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        rospy.loginfo("Controller init finished")
        pass

    def control(self, current_vel_lin_x, current_vel_ang_z, target_vel_lin_x, target_vel_ang_z, dbw_enabled, delta_t):
        
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake_torque, steering
        rospy.loginfo("Control function started")
        
        # Initialize throttle, brake & steering output
        throttle = brake_torque = steering = 0
        
        # If DBW is disabled reset PID and output control values = 0
        if dbw_enabled == 0:
            self.throttle_controller.reset()
            self.steering_controller.reset()
            return throttle, brake_torque, steering # initial values, all 0
        
        # LowPassFilter current velocity
        current_vel_lin_x = self.vel_lowpassfiltered.filt(current_vel_lin_x)
        if target_vel_lin_x is None:
            rospy.loginfo("Error: target_vel_lin_x is None")
            
        if current_vel_lin_x is None:
            rospy.loginfo("Error: current_vel_lin_x is None")

            
        lin_x_vel_error = target_vel_lin_x - current_vel_lin_x
        rospy.loginfo("lin_x_vel_error: %.2f, target_vel_lin_x: %.2f, current_vel_lin_x: %.2f", lin_x_vel_error, target_vel_lin_x, current_vel_lin_x)

        # Calculate Throttle Control Value
        throttle = self.throttle_controller.step(lin_x_vel_error, delta_t)
        self.last_vel = current_vel_lin_x
        
        # Calculate Brake Value and potentially overwrite throttle value to 0
        if current_vel_lin_x < 0.1 and target_vel_lin_x == 0.0:
            rospy.loginfo("Vehicle in standstill and target velocity = 0 --> Brake against Creep needed")
            # Vehicle is in standstill (<0.1 m/s) and shall reside there, apply brake and release throttle
            throttle = 0.0
            brake_torque = brake_torque_to_keep_zero_speed # this torque is needed to brake against the creep and potential slopes due to automatic transmission
        elif lin_x_vel_error < 0  and throttle < 0.1:
            # Set speed is smaller than current speed and throttle is just a little bit engaged
            rospy.loginfo("Braking needed!")
            throttle = 0.0
            decel_req = min(abs(lin_x_vel_error) / delta_t, self.decel_limt)
            brake_torque = abs(decel_req) * self.vehicle_mass * self.wheel_radius
        
        # Calculate  steering value with Yaw Controller
        rospy.loginfo("Current x: %.2f, Target x: %.2f, Current z: %.2f, Target z: %.2f", current_vel_lin_x, target_vel_lin_x, current_vel_ang_z, target_vel_ang_z)
        steering = self.yaw_controller.get_steering(target_vel_lin_x, target_vel_ang_z, current_vel_lin_x)
        ang_z_vel_error = target_vel_ang_z - current_vel_ang_z
        #rospy.loginfo("ang_z_vel_error: %.2f, target_vel_ang_z: %.2f, current_vel_ang_z: %.2f", ang_z_vel_error, target_vel_ang_z, current_vel_ang_z)
        #steering = self.steering_controller.step(ang_z_vel_error, delta_t)
              
        rospy.loginfo("Control function finished")
        return throttle, brake_torque, steering

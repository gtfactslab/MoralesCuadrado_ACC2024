# Final Code for ACC Submission: newton raphson controller with I-CBFs, ready for simulation and hardware testing on two predictors (linear and nonlinear).
# 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry, TrajectorySetpoint, RcChannels
from std_msgs.msg import Float64, Float64MultiArray, String

import sympy as smp
from scipy.integrate import quad
from scipy.linalg import expm
from sympy import * 
import numpy as np

from tf_transformations import euler_from_quaternion
import time

from math import sqrt
import math as m

import csv
import ctypes



class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')


        class Vector9x1(ctypes.Structure):
            _fields_ = [
                ('x', ctypes.c_double),
                ('y', ctypes.c_double),
                ('z', ctypes.c_double),
                ('vx', ctypes.c_double),
                ('vy', ctypes.c_double),
                ('vz', ctypes.c_double),
                ('roll', ctypes.c_double),
                ('pitch', ctypes.c_double),
                ('yaw', ctypes.c_double),
            ]

        # Load the C shared library
        self.my_library = ctypes.CDLL('/home/factslabegmc//newtonraphson_final_ws/src/newton_raphson/newton_raphson/libwork.so')  # Update the library filename

        # Set argument and return types for the function
        self.my_library.performCalculations.argtypes = [
            ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int
        ]
        self.my_library.performCalculations.restype = ctypes.POINTER(Vector9x1)


    

        # Figure out if in simulation or hardware mode to set important variables to the appropriate values
        self.sim = bool(int(input("Are you using the simulator? Write 1 for Sim and 0 for Hardware: ")))
        # print(f"sim: {self.sim}")
        self.nr_time_el = []

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # Create Publishers
        # Publishers for Setting to Offboard Mode and Arming/Diasarming/Landing/etc
        self.offboard_control_mode_publisher = self.create_publisher( #publishes offboard control heartbeat
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher( #publishes vehicle commands (arm, offboard, disarm, etc)
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Publishers for Sending Setpoints in Offboard Mode: 1) Body Rates and Thrust, 2) Position and Yaw 
        self.rates_setpoint_publisher = self.create_publisher( #publishes body rates and thrust setpoint
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher( #publishes trajectory setpoint
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Publisher for Logging States, Inputs, and Reference Trajectories for Data Analysis
        self.state_input_ref_log_publisher_ = self.create_publisher( #publishes log of states and input
            Float64MultiArray, '/state_input_ref_log', 10)
        self.state_input_ref_log_msg = Float64MultiArray() #creates message for log of states and input
        
        # Publisher for Logging States, Inputs, and Reference Trajectories for Data Analysis
        self.nr_time_elapsed_publisher = self.create_publisher( #publishes log of states and input
            Float64MultiArray, '/nr_elapsed_time_pub', 10)
        self.nr_time_elapsed_msg = Float64MultiArray() #creates message for log of states and input
        


        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription( #subscribes to odometry data (position, velocity, attitude)
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription( #subscribes to vehicle status (arm, offboard, disarm, etc)
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
    
        self.offboard_mode_rc_switch_on = True if self.sim else False #Offboard mode starts on if in Sim, turn off and wait for RC if in hardware
        self.rc_channels_subscriber = self.create_subscription( #subscribes to rc_channels topic for software "killswitch" to make sure we'd like position vs offboard vs land mode
            RcChannels, '/fmu/out/rc_channels', self.rc_channel_callback, qos_profile
        )


        # Initialize variables:
        self.time_before_land = 30.0
        print(f"time_before_land: {self.time_before_land}")
        self.offboard_setpoint_counter = 0 #helps us count 10 cycles of sending offboard heartbeat before switching to offboard mode and arming
        self.vehicle_status = VehicleStatus() #vehicle status variable to make sure we're in offboard mode before sending setpoints

        self.T0 = time.time() # initial time of program
        self.timefromstart = time.time() - self.T0 # time from start of program initialized and updated later to keep track of current time in program

        self.g = 9.806 #gravity
        self.T_lookahead = .8 #lookahead time for prediction and reference tracking in NR controller
        #mass of drone (kg) used in linearized model function, there's a slighly different mass used for gravity to offset constant error in z-direction
        

        # The following 3 variables are used to convert between force and throttle commands (iris gazebo simulation)
        self.motor_constant_ = 0.00000584 #iris gazebo simulation motor constant
        self.motor_velocity_armed_ = 100 #iris gazebo motor velocity when armed
        self.motor_input_scaling_ = 1000.0 #iris gazebo simulation motor input scaling

        # Initialize first input (throttle) for hover at origin
        if self.sim:
            print("Using simulator throttle from force conversion function")
            self.m = 1.535 #set simulation mass from iris model sdf for linearized model calculations
            self.gravmass = 1.55 #offset gravitational mass to change throttle over/under compensation to make trajectory more accurate in z-direction
        elif not self.sim:
            print("Using hardware throttle from force conversion function and certain trajectories will not be available")
            self.m = 1.69 #weighed the drone with everything on it including battery: 3lb 11.7oz -> 3.73lbs -> 1.69kg
            self.gravmass = 1.73

        # exit(0)
        self.u0 = np.array([[self.get_throttle_command_from_force(-1*self.m*self.g), 0, 0, 0]]).T
        # exit(0)

        self.linpred = bool(int(input("Linear or Nonlinear Predictor? Write 1 for Linear and 0 for Nonlinear: ")))
        if self.linpred:
            print("Using Linear Predictor")
            self.linearized_model() #Calculate Linearized Model Matrices
        else:
            self.C = self.observer_matrix() #Calculate Observer Matrix


        # Initialize Matrices for Linearized Model Prediction and NR Input Calculation (eAT, int_eATB, int_eAT, C, jac_inv)
        # self.linearized_model() #Calculate Linearized Model Matrices
        # self.jac_inv = np.linalg.inv(self.getyorai_gJac_linear_predict()) #Calculate Inverse Jacobian of Linearized Model Matrices
        # print(f"jac_inv: {self.jac_inv}")
        # print(f"jac_inv.shape: {self.jac_inv.shape}")
        # exit(0)

        #Create Function @ {1/self.offboard_timer_period}Hz (in my case should be 10Hz/0.1 period) to Publish Offboard Control Heartbeat Signal
        self.offboard_timer_period = 0.1
        self.timer = self.create_timer(self.offboard_timer_period, self.offboard_mode_timer_callback)
        # exit(0)

        # Create Function at {1/self.newton_raphson_timer_period}Hz (in my case should be 100Hz/0.01 period) to Send NR Control Input
        self.newton_raphson_timer_period = 0.01
        self.timer = self.create_timer(self.newton_raphson_timer_period, self.newton_raphson_timer_callback)



    # The following 4 functions all call publish_vehicle_command to arm/disarm/land/ and switch to offboard mode
    # The 5th function publishes the vehicle command
    # The 6th function checks if we're in offboard mode
    def arm(self): #1. Sends arm command to vehicle via publish_vehicle_command function
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self): #2. Sends disarm command to vehicle via publish_vehicle_command function
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self): #3. Sends offboard command to vehicle via publish_vehicle_command function
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self): #4. Sends land command to vehicle via publish_vehicle_command function
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_vehicle_command(self, command, **params) -> None: #5. Called by the above 4 functions to send parameter/mode commands to the vehicle
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def vehicle_status_callback(self, vehicle_status): #6. This function helps us check if we're in offboard mode before we start sending setpoints
        """Callback function for vehicle_status topic subscriber."""
        # print('vehicle status callback')
        self.vehicle_status = vehicle_status

    def rc_channel_callback(self, rc_channels):
        """Callback function for RC Channels to create a software 'killswitch' depending on our flight mode channel (position vs offboard vs land mode)"""
        # print('rc channel callback')
        mode_channel = 5
        flight_mode = rc_channels.channels[mode_channel-1] # +1 is offboard everything else is not offboard
        self.offboard_mode_rc_switch_on = True if flight_mode >= 0.75 else False


    # The following 2 functions are used to publish offboard control heartbeat signals
    def publish_offboard_control_heartbeat_signal2(self): #1)Offboard Signal2 for Returning to Origin with Position Control
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal1(self): #2)Offboard Signal1 for Newton-Rapshon Body Rate Control
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)



# ~~ The remaining functions are all intimately related to the Newton-Rapshon Control Algorithm ~~

    # The following 2 functions are used to convert between force and throttle commands
    def get_throttle_command_from_force(self, collective_thrust): #Converts force to throttle command
        collective_thrust = -collective_thrust
        print(f"collective_thrust: {collective_thrust}")
        if self.sim:
            motor_speed = sqrt(collective_thrust / (4.0 * self.motor_constant_))
            throttle_command = (motor_speed - self.motor_velocity_armed_) / self.motor_input_scaling_
            return -throttle_command
        if not self.sim:
            # print('using hardware throttle from force conversion function')
            a = 0.00705385408507030
            b = 0.0807474474438391
            c = 0.0252575818743285

            # equation form is a*x + b*sqrt(x) + c = y
            throttle_command = a*collective_thrust + b*sqrt(collective_thrust) + c
            return -throttle_command

    def get_force_from_throttle_command(self, throttle_command): #Converts throttle command to force
        throttle_command = -throttle_command
        print(f"throttle_command: {throttle_command}")
        if self.sim:
            motor_speed = (throttle_command * self.motor_input_scaling_) + self.motor_velocity_armed_
            collective_thrust = 4.0 * self.motor_constant_ * motor_speed ** 2
            return -collective_thrust
        
        if not self.sim:
            # print('using hardware force from throttle conversion function')
            a = 19.2463167420814
            b = 41.8467162352942
            c = -7.19353022443441

            # equation form is a*x^2 + b*x + c = y
            collective_thrust = a*throttle_command**2 + b*throttle_command + c
            return -collective_thrust
        

    def angle_wrapper(self, angle):
        angle += m.pi
        angle = angle % (2 * m.pi)  # Ensure the angle is between 0 and 2π
        if angle > m.pi:            # If angle is in (π, 2π], subtract 2π to get it in (-π, 0]
            angle -= 2 * m.pi
        if angle < -m.pi:           # If angle is in (-2π, -π), add 2π to get it in (0, π]
            angle += 2 * m.pi
        return -angle
    
    def vehicle_odometry_callback(self, msg): # Odometry Callback Function Yields Position, Velocity, and Attitude Data
        """Callback function for vehicle_odometry topic subscriber."""
        # print("AT ODOM CALLBACK")
        (self.yaw, self.pitch, self.roll) = euler_from_quaternion(msg.q)
        # print(f"old_yaw: {self.yaw}")
        self.yaw = self.angle_wrapper(self.yaw)
        self.pitch = -1 * self.pitch #pitch is negative of the value in gazebo bc of frame difference

        self.p = msg.angular_velocity[0]
        self.q = msg.angular_velocity[1]
        self.r = msg.angular_velocity[2]

        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = 1 * msg.position[2] # z is negative of the value in gazebo bc of frame difference

        self.vx = msg.velocity[0]
        self.vy = msg.velocity[1]
        self.vz = 1 * msg.velocity[2] # vz is negative of the value in gazebo bc of frame difference

        # print(f"Roll: {self.roll}")
        # print(f"Pitch: {self.pitch}")
        # print(f"Yaw: {self.yaw}")
        
        self.stateVector = np.array([[self.x, self.y, self.z, self.vx, self.vy, self.vz, self.roll, self.pitch, self.yaw]]).T 
        self.nr_state = np.array([[self.x, self.y, self.z, self.yaw]]).T
        self.odom_rates = np.array([[self.p, self.q, self.r]]).T



    def publish_rates_setpoint(self, thrust: float, roll: float, pitch: float, yaw: float): #Publishes Body Rate and Thrust Setpoints
        """Publish the trajectory setpoint."""
        msg = VehicleRatesSetpoint()
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw = float(yaw)
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = 1* float(thrust)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.rates_setpoint_publisher.publish(msg)
        
        # print("in publish rates setpoint")
        # self.get_logger().info(f"Publishing rates setpoints [r,p,y]: {[roll, pitch, yaw]}")
        print(f"Publishing rates setpoints [thrust, r,p,y]: {[thrust, roll, pitch, yaw]}")

    def publish_position_setpoint(self, x: float, y: float, z: float): #Publishes Position and Yaw Setpoints
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")


# ~~ The following 2 functions are the main functions that run at 10Hz and 100Hz ~~
    def offboard_mode_timer_callback(self) -> None: # ~~Runs at 10Hz and Sets Vehicle to Offboard Mode  ~~
        """Offboard Callback Function for The 10Hz Timer."""
        # print("In offboard timer callback")

        if self.offboard_mode_rc_switch_on: #integration of RC 'killswitch' for offboard deciding whether to send heartbeat signal, engage offboard, and arm
            if self.timefromstart <= self.time_before_land:
                self.publish_offboard_control_heartbeat_signal1()
            elif self.timefromstart > self.time_before_land:
                self.publish_offboard_control_heartbeat_signal2()


            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()
            if self.offboard_setpoint_counter < 11:
                self.offboard_setpoint_counter += 1

        else:
            print("Offboard Callback: RC Flight Mode Channel 5 Switch Not Set to Offboard (-1: position, 0: offboard, 1: land) ")
            self.offboard_setpoint_counter = 0



    def newton_raphson_timer_callback(self) -> None: # ~~This is the main function that runs at 100Hz and Administrates Calls to Every Other Function ~~
        # print("NR_Callback")
        if self.offboard_mode_rc_switch_on: #integration of RC 'killswitch' for offboard deciding whether to send heartbeat signal, engage offboard, and arm
            self.timefromstart = time.time()-self.T0 #update curent time from start of program for reference trajectories and for switching between NR and landing mode
            

            print(f"--------------------------------------")
            # print(self.vehicle_status.nav_state)
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                print(f"NR_callback- timefromstart: {self.timefromstart}")
                print("IN OFFBOARD MODE")

                if self.timefromstart <= self.time_before_land: # wardi controller for first {self.time_before_land} seconds
                    print("Newton-Raphson Control")
                    self.newton_raphson_control()

                elif self.timefromstart > self.time_before_land: #then land at origin and disarm
                    print("BACK TO SPAWN")
                    self.publish_position_setpoint(0.0, 0.0, -0.3)
                    print(f"self.x: {self.x}, self.y: {self.y}, self.z: {self.z}")
                    if abs(self.x) < 0.1 and abs(self.y) < 0.1 and abs(self.z) <= 0.50:
                        print("Switching to Land Mode")
                        self.land()

            if self.timefromstart > self.time_before_land:
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                        print("IN LAND MODE")
                        if abs(self.z) <= .18:
                            print("Disarming and Exiting Program")
                            self.disarm()
                            # # Specify the file name
                            # csv_file = 'example.csv'
                            # print(self.nr_time_el)
                            # # Open the .csv file for writing
                            # with open(csv_file, mode='w', newline='') as file:
                            #     writer = csv.writer(file)  # Create a CSV writer

                            #     # Write the data to the CSV file
                            #     for row in self.nr_time_el:
                            #         writer.writerow(row)

                            # print(f'Data has been written to {csv_file}')
                            exit(0)
            print(f"--------------------------------------")
            print("\n\n")
        else:
            print("NR_Callback: Channel 11 Switch Not Set to Offboard")

    
# ~~ From here down are the functions that actually calculate the control input ~~
    def newton_raphson_control(self): # Runs Newton-Rapshon Control Algorithm Structure
        print(f"NR_State: {self.nr_state}")


        # Change the previous input from throttle to force for NR calculations that require the previous input
        old_throttle = self.u0[0][0]
        old_force = self.get_force_from_throttle_command(old_throttle)
        lastinput_using_force = np.vstack([old_force, self.u0[1:]])

        # Calculate the current reference trajectory

        # reffunc = self.circle_vert_ref_func() #FIX THIS SHIT B4 RUNNING
        # reffunc = self.circle_horz_ref_func()
        # reffunc = self.fig8_horz_ref_func()
        # reffunc = self.fig8_vert_ref_func_short()
        reffunc = self.fig8_vert_ref_func_tall()
        # reffunc = self.hover_ref_func(1)
        # if self.timefromstart <= 10.0:
        #     reffunc = self.hover_ref_func(5)
        # elif self.timefromstart > 10.0:
        #     reffunc = self.hover_ref_func(1)

        print(f"reffunc: {reffunc}")

        # Calculate the Newton-Rapshon control input and transform the force into a throttle command for publishing to the vehicle
        new_u = self.get_new_NR_input(lastinput_using_force, reffunc)
        new_force = new_u[0][0]        
        print(f"new_force: {new_force}")

        new_throttle = self.get_throttle_command_from_force(new_force)
        new_roll_rate = new_u[1][0]
        new_pitch_rate = new_u[2][0]
        new_yaw_rate = new_u[3][0]



        # Build the final input vector to save as self.u0 and publish to the vehicle via publish_rates_setpoint:
        final = [new_throttle, new_roll_rate, new_pitch_rate, new_yaw_rate]
        # final = [throttle_lim, roll_lim, pitch_lim, 0.0]


        # print(f"final: {final}")
        current_input_save = np.array(final).reshape(-1, 1)
        print(f"newInput: \n{current_input_save}")
        self.u0 = current_input_save

        # self.publish_rates_setpoint(.71, 0.0, 0.0, 0.0)
        self.publish_rates_setpoint(final[0], final[1], final[2], final[3])

        # Log state, input, and reference
        # print(f"self.x: {self.x}, self.y: {self.y}, self.z: {self.z}, self.yaw: {self.yaw}")
        # print(f"throttle: {final[0]}, roll: {final[1]}, pitch: {final[2]}, yaw: {final[3]}")
        # print(f"ref_x: {reffunc[0][0]}, ref_y: {reffunc[1][0]}, ref_z: {reffunc[2][0]}, ref_yaw: {reffunc[3][0]}")
        # exit(0)
        # print(f'{[float(self.x), float(self.y), float(self.z), float(self.yaw)]}, \n {[float(final[0]), float(final[1]), float(final[2]), float(final[3])]} \n {[float(reffunc[0][0]), float(reffunc[1][0]), float(reffunc[2][0]), float(reffunc[3][0])]}')
        self.state_input_ref_log_msg.data = [float(self.x), float(self.y), float(self.z), float(self.yaw), float(final[0]), float(final[1]), float(final[2]), float(final[3]), float(reffunc[0][0]), float(reffunc[1][0]), float(reffunc[2][0]), float(reffunc[3][0])]
        # exit(0)
        self.state_input_ref_log_publisher_.publish(self.state_input_ref_log_msg)
        # exit(0)

    def get_new_NR_input(self, lastinput, reffunc): #Calculates Newton-Rapshon Control Input "new_u"
        nrt0 = time.time() # time before NR calculations
        # newton-raphson control input calculation without speed-up parameter (udot = inv(dg/du) * (yref - ypred) = NR) (alpha comes later)


        print("\n")
        print("#################################################")
        t1 = time.time() # time before prediction
        if self.linpred:
            print(f"Using linear prediction")
            pred = self.getyorai_g_linear_predict(lastinput) # predicts system output state T_lookahead seconds in the future
        else:
            print(f"Using numerically integrated nonlinear model prediction")
            pred = self.get_nonlin_predict3(lastinput) # predicts system output state T_lookahead seconds in the future

        # print(f"time to predict: {time.time()-t1}")
        # print(f"pred: {pred[0:]}")
        print("#################################################")
        print("\n")

        pred=pred
        err = reffunc-pred # calculates error between reference and predicted state
        dgdu_inv = self.jac_inv # inverse of jacobian of system output w.r.t. input
        NR = dgdu_inv @ err # calculates newton-raphson control input without speed-up parameter

        # print(f"dgdu_inv: {dgdu_inv}")
        # print(f"pred: {pred}")
        # print(f"err: {err}")
        # print(f"NR: {NR}")

###################################################################################
# ~~ CBF IMPLEMENTATION THAT DIRECTS udot TO KEEP INPUT u WITHIN SAFE REGION ~~

    # Set up CBF parameters:

        # Get current thrust (force) and rates
        curr_thrust = lastinput[0][0]
        curr_roll_rate = lastinput[1][0]
        curr_pitch_rate = lastinput[2][0]
        curr_yaw_rate = lastinput[3][0]

        # Get current newton-raphson udot value we just calculated that we want to direct towards safe region(NR = (dg/du)^-1 * (yref - ypred)) (before alpha tuning)
        phi = NR
        phi_thrust = phi[0][0]
        phi_roll_rate = phi[1][0]
        phi_pitch_rate = phi[2][0]
        phi_yaw_rate = phi[3][0]

# CBF FOR THRUST
        v_thrust = 0.0 # influence value initialized to 0 as default for if no CBF is needed
        gamma = 1.0 # CBF parameter
        thrust_max = -0.5 # max thrust (force) value to limit thrust to
        thrust_min = -27.0 # min thrust (force) value to limit thrust to

        # print(f"curr_thrust: {curr_thrust}")
        # Optimization procedure for CBF
        if curr_thrust >= 0:
            zeta = gamma * (thrust_max - curr_thrust) - phi_thrust
            if zeta < 0:
                v_thrust = zeta

        if curr_thrust < 0:
            zeta = -gamma * (-thrust_min + curr_thrust) - phi_thrust
            if zeta > 0:
                v_thrust = zeta
        # print(f"zetathrust: {zeta}")

# SET UP CBF FOR RATES
        rates_max_abs = 0.8 # max absolute value of roll, pitch, and yaw rates to limit rates to
        rates_max = rates_max_abs 
        rates_min = -rates_max_abs

    # CBF FOR ROLL
        v_roll = 0.0 # influence value initialized to 0 as default for if no CBF is needed
        gamma = 1.0 # CBF parameter

        # Optimization procedure for CBF
        if curr_roll_rate >= 0:
            zeta = gamma * (rates_max - curr_roll_rate) - phi_roll_rate
            if zeta < 0:
                v_roll = zeta
        elif curr_roll_rate < 0:
            zeta = -gamma * (-rates_min + curr_roll_rate) - phi_roll_rate
            if zeta > 0:
                v_roll = zeta

    # CBF FOR PITCH
        v_pitch = 0.0 # influence value initialized to 0 as default for if no CBF is needed
        gamma = 1.0 # CBF parameter

        # Optimization procedure for CBF
        if curr_pitch_rate >= 0:
            zeta = gamma * (rates_max - curr_pitch_rate) - phi_pitch_rate
            if zeta < 0:
                v_pitch = zeta
        elif curr_pitch_rate < 0:
            zeta = -gamma * (-rates_min + curr_pitch_rate) - phi_pitch_rate
            if zeta > 0:
                v_pitch = zeta

        v = np.array([[v_thrust, v_roll, v_pitch, 0]]).T
###################################################################################

        # print(f"v: {v}")
        # calculate NR input with CBF
        udot = phi + v # udot = { inv(dg/du) * (yref - ypred) } + v = NR + v = newton-raphson control input + CBF adjustment
        change_u = udot * self.newton_raphson_timer_period #crude integration of udot to get u (maybe just use 0.02 as period)

        # alpha=np.array([[10,10,10,10]]).T
        # alpha=np.array([[20,30,30,30]]).T # Speed-up parameter (maybe play with uniform alpha values rather than ones that change for each input)
        # alpha=np.array([[40,40,40,40]]).T # Speed-up parameter (maybe play with uniform alpha values rather than ones that change for each input)
        alpha=np.array([[45,45,45,45]]).T # Speed-up parameter (maybe play with uniform alpha values rather than ones that change for each input)

        update_u = alpha * change_u
        u = lastinput + alpha * change_u # u_new = u_old + alpha * change_u

        # print(f"udot: {udot}")
        # print(f"change_u: {change_u}")
        # print(f"update_u: {update_u}")
        # print(f"lastinput: {lastinput}")
        # print(f"u: {u}")
        # exit(0)
        nrtf = time.time() # time after NR calculations
        nr_time_elapsed = nrtf-nrt0 # time elapsed during NR calculations
        self.nr_time_el.append(nr_time_elapsed)
        print(f"NR_time_elapsed: {nr_time_elapsed}")
        self.nr_time_elapsed_msg.data = [float(nr_time_elapsed)]
        self.nr_time_elapsed_publisher.publish(self.nr_time_elapsed_msg)
        # # exit(0)
        return u
    
    def get_nonlin_predict3(self, lastinput):
        g = self.g
        m = self.gravmass
        curr_thrust = -lastinput[0][0]
        curr_rolldot = lastinput[1][0]
        curr_pitchdot = lastinput[2][0]
        curr_yawdot = lastinput[3][0]

        curr_x = self.x
        curr_y = self.y
        curr_z = self.z
        curr_vx = self.vx
        curr_vy = self.vy
        curr_vz = self.vz
        curr_roll = self.roll
        curr_pitch = self.pitch
        curr_yaw = self.yaw

        T_lookahead = 0.8
        integration_step = 0.1
        integrations = T_lookahead / integration_step
        integrations_int = int(integrations)



        # Call the C function for non-linear prediction of outputs
        output_ptr = self.my_library.performCalculations(
            g, m, curr_thrust, curr_rolldot, curr_pitchdot, curr_yawdot,
            curr_x, curr_y, curr_z, curr_vx, curr_vy, curr_vz, curr_roll, curr_pitch, curr_yaw, integration_step, integrations_int
        )
        # Dereference the output pointer to access the vector
        nonlin_pred = output_ptr.contents
        x = nonlin_pred.x
        y = nonlin_pred.y
        z = nonlin_pred.z
        vx = nonlin_pred.vx
        vy = nonlin_pred.vy
        vz = nonlin_pred.vz
        roll = nonlin_pred.roll
        pitch = nonlin_pred.pitch
        yaw = nonlin_pred.yaw
        pred = np.array([[x, y, z, vx, vy, vz, roll, pitch, yaw]]).T
        outputs = self.C @ pred
        # Free the allocated memory
        self.my_library.free(output_ptr)

        # Call the C function 4 more times to get jacobian wrt inputs
        epsilon = 1e-5

        pertub_thrust = curr_thrust + epsilon
        output_ptr = self.my_library.performCalculations(
            g, m, pertub_thrust, curr_rolldot, curr_pitchdot, curr_yawdot,
            curr_x, curr_y, curr_z, curr_vx, curr_vy, curr_vz, curr_roll, curr_pitch, curr_yaw, integration_step, integrations_int
        )
        nonlin_pred = output_ptr.contents
        x_var1 = nonlin_pred.x
        y_var1 = nonlin_pred.y
        z_var1 = nonlin_pred.z
        yaw_var1 = nonlin_pred.yaw
        self.my_library.free(output_ptr)
        dpdu1 = np.array([[(x_var1 - x) / epsilon,
                           (y_var1 - y) / epsilon,
                           (z_var1 - z) / epsilon,
                           (yaw_var1 - yaw) / epsilon]]).T


        perturb_rolldot = curr_rolldot + epsilon
        output_ptr = self.my_library.performCalculations(
            g, m, curr_thrust, perturb_rolldot, curr_pitchdot, curr_yawdot,
            curr_x, curr_y, curr_z, curr_vx, curr_vy, curr_vz, curr_roll, curr_pitch, curr_yaw, integration_step, integrations_int
        )
        nonlin_pred = output_ptr.contents
        x_var2 = nonlin_pred.x
        y_var2 = nonlin_pred.y
        z_var2 = nonlin_pred.z
        yaw_var2 = nonlin_pred.yaw
        self.my_library.free(output_ptr)
        dpdu2 = np.array([[(x_var2 - x) / epsilon,
                           (y_var2 - y) / epsilon,
                           (z_var2 - z) / epsilon,
                           (yaw_var2 - yaw) / epsilon]]).T



        perturb_pitchdot = curr_pitchdot + epsilon
        output_ptr = self.my_library.performCalculations(
            g, m, curr_thrust, curr_rolldot, perturb_pitchdot, curr_yawdot,
            curr_x, curr_y, curr_z, curr_vx, curr_vy, curr_vz, curr_roll, curr_pitch, curr_yaw, integration_step, integrations_int
        )
        nonlin_pred = output_ptr.contents
        x_var3 = nonlin_pred.x
        y_var3 = nonlin_pred.y
        z_var3 = nonlin_pred.z
        yaw_var3 = nonlin_pred.yaw
        self.my_library.free(output_ptr)
        dpdu3 = np.array([[(x_var3 - x) / epsilon,
                           (y_var3 - y) / epsilon,
                           (z_var3 - z) / epsilon,
                           (yaw_var3 - yaw) / epsilon]]).T


        perturb_yawdot = curr_yawdot + epsilon
        output_ptr = self.my_library.performCalculations(
            g, m, curr_thrust, curr_rolldot, curr_pitchdot, perturb_yawdot,
            curr_x, curr_y, curr_z, curr_vx, curr_vy, curr_vz, curr_roll, curr_pitch, curr_yaw, integration_step, integrations_int
        )
        nonlin_pred = output_ptr.contents
        x_var4 = nonlin_pred.x
        y_var4 = nonlin_pred.y
        z_var4 = nonlin_pred.z
        yaw_var4 = nonlin_pred.yaw
        self.my_library.free(output_ptr)
        dpdu4 = np.array([[(x_var4 - x) / epsilon,
                           (y_var4 - y) / epsilon,
                           (z_var4 - z) / epsilon,
                           (yaw_var4 - yaw) / epsilon]]).T



        jac_u = np.hstack([dpdu1, dpdu2, dpdu3, dpdu4])
        # print(f"dpdu3: {dpdu3}")
        # print(f"jac_u: {jac_u}")
        # print(f"jac_u.shape: {jac_u.shape}")
        # make the third column of inverse jacobian negative
        inv_jac = np.linalg.inv(jac_u)
        # print(f"inv_jac: {inv_jac}")
        inv_jac[:, 2] = -inv_jac[:, 2]
        # print(f"inv_jac: {inv_jac}")

        self.jac_inv = inv_jac
        # exit(0)
        return outputs
    





    # def get_nonlin_predict2(self, lastinput):
    #     g = self.g
    #     m = self.gravmass
    #     curr_thrust = -lastinput[0][0]
    #     curr_rolldot = lastinput[1][0]
    #     curr_pitchdot = lastinput[2][0]
    #     curr_yawdot = lastinput[3][0]

    #     curr_x = self.x
    #     curr_y = self.y
    #     curr_z = self.z
    #     curr_vx = self.vx
    #     curr_vy = self.vy
    #     curr_vz = self.vz
    #     curr_roll = self.roll
    #     curr_pitch = self.pitch
    #     curr_yaw = self.yaw

    #     T_lookahead = 0.8
    #     integration_step = 0.1
    #     integrations = T_lookahead / integration_step
    #     integrations = int(integrations)

    #     cumm_change_x = 0.0
    #     cumm_change_y = 0.0
    #     cumm_change_z = 0.0
    #     cumm_change_vx = 0.0
    #     cumm_change_vy = 0.0
    #     cumm_change_vz = 0.0
    #     cumm_change_roll = 0.0
    #     cumm_change_pitch = 0.0
    #     cumm_change_yaw = 0.0

    #     xdot = curr_vx
    #     ydot = curr_vy
    #     zdot = curr_vz

    #     vxdot = -(curr_thrust/m) * (sin(curr_roll)*sin(curr_yaw) + cos(curr_roll)*cos(curr_yaw)*sin(curr_pitch));
    #     vydot = -(curr_thrust/m) * (cos(curr_roll)*sin(curr_yaw)*sin(curr_pitch) - cos(curr_yaw)*sin(curr_roll));
    #     vzdot = g - (curr_thrust/m) * (cos(curr_roll)*cos(curr_pitch));
    
    #     rolldot = curr_rolldot
    #     pitchdot = curr_pitchdot
    #     yawdot = curr_yawdot

    #     roll = curr_roll
    #     pitch = curr_pitch
    #     yaw = curr_yaw
    #     change_vx = 0
    #     change_vy = 0
    #     change_vz = 0



    #     for _ in range(integrations):
    #         change_x = (xdot+cumm_change_vx) * integration_step;
    #         change_y = (ydot+cumm_change_vy) * integration_step;
    #         change_z = (zdot+cumm_change_vz) * integration_step;
    #         change_vx = vxdot * integration_step;
    #         change_vy = vydot * integration_step;
    #         change_vz = vzdot * integration_step;
    #         change_roll = rolldot * integration_step;
    #         change_pitch = pitchdot * integration_step;
    #         change_yaw = yawdot * integration_step;


    #         roll = roll + change_roll;
    #         pitch = pitch + change_pitch;
    #         yaw =  yaw + change_yaw;

    #         sr = sin(roll)
    #         sy = sin(yaw)
    #         sp = sin(pitch)

    #         cr = cos(roll)
    #         cp = cos(pitch)
    #         cy = cos(yaw)
            
            
    #         vxdot = -(curr_thrust/m) * (sr*sy + cr*cy*sp);
    #         vydot = -(curr_thrust/m) * (cr*sy*sp - cy*sr);
    #         vzdot = g - (curr_thrust/m) * (cr*cp);


    #         cumm_change_x = cumm_change_x + change_x;
    #         cumm_change_y = cumm_change_y + change_y; 
    #         cumm_change_z = cumm_change_z + change_z; 
    #         cumm_change_vx = cumm_change_vx + change_vx; 
    #         cumm_change_vy = cumm_change_vy + change_vy; 
    #         cumm_change_vz = cumm_change_vz + change_vz; 
    #         cumm_change_roll = cumm_change_roll + change_roll; 
    #         cumm_change_pitch = cumm_change_pitch + change_pitch; 
    #         cumm_change_yaw = cumm_change_yaw + change_yaw;




    #     x = curr_x + cumm_change_x
    #     y = curr_y + cumm_change_y
    #     z = curr_z + cumm_change_z
        
    #     vx = curr_vx + cumm_change_vx
    #     vy = curr_vy + cumm_change_vy
    #     vz = curr_vz + cumm_change_vz
    
    #     roll = curr_roll + cumm_change_roll;
    #     pitch = curr_pitch + cumm_change_pitch;
    #     yaw = curr_yaw + cumm_change_yaw;

    #     nonlin_pred = np.array([[x, y, z, vx, vy, vz, roll, pitch, yaw]]).T
    #     outputs = self.C @ nonlin_pred
    #     return outputs

        
# ~~ The following functions produce the linearized model matrices that we use for prediction and NR input calculation ~~
    def getyorai_g_linear_predict(self, currinput): #Calculates Linearized System Output Prediction ([x,y,z,yaw])
        gravity = np.array([[self.gravmass*self.g,0,0,0]]).T #gravity vector that counteracts input vector: [-mg, 0, 0, 0]
        lin_pred = self.eAT@self.stateVector + self.int_eATB @ (currinput + gravity) # xdot = eAT*x(t) + int_eATB*(u-gravity)
        yorai_g = self.C @ lin_pred # y(t) = C*x(t) = [x,y,z,yaw]
        return yorai_g

    def getyorai_gJac_linear_predict(self): #Calculates Jacobian of Linearized System Output Prediction for Newton-Raphson Input Calc : udot = a*inv(dg/du)(err)
        Jac = np.concatenate((self.int_eATB[0:3, :], self.int_eATB[-1:, :]), axis=0) # dg/du = C * int_eATB
        return Jac
    
    def observer_matrix(self):
        C = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1]])
        return C
    
    def linearized_model(self): #Calculates Linearized Model Matrices for Prediction (eAT, int_eATB, int_eAT, C)

        # x(t) = eAT*x(t0) + int_eATB*u(t) with u(t) = u over T = T_lookahead seconds
        # simplifies to x(t) = eAT*x(t) + int_eATB*(u - gravity) in our implementation as seen in getyorai_g_linear_predict
        # y(t) = C*x(t)

        A = smp.Matrix(
            [
                [0, 0, 0,    1, 0, 0,     0,   0, 0],
                [0, 0, 0,    0, 1, 0,     0,   0, 0],
                [0, 0, 0,    0, 0, 1,     0,   0, 0],

                [0, 0, 0,    0, 0, 0,     0,-1*self.g, 0],
                [0, 0, 0,    0, 0, 0,     self.g,   0, 0],
                [0, 0, 0,    0, 0, 0,     0,   0, 0],

                [0, 0, 0,    0, 0, 0,     0,   0, 0],
                [0, 0, 0,    0, 0, 0,     0,   0, 0],
                [0, 0, 0,    0, 0, 0,     0,   0, 0],

            ]
            )
        
        eAT = smp.exp(A*self.T_lookahead)

        B = smp.Matrix(
            [
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],

                [1/self.m, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
            )

        A = np.array(A).astype(np.float64)
        int_eAT = np.zeros_like(A)
        rowcol = int_eAT.shape[0]
        for row in range(rowcol):
            for column in range(rowcol):
                f = lambda x: expm(A*(self.T_lookahead-x))[row,column]
                int_eAT[row,column] = quad(f, 0, self.T_lookahead)[0]


        int_eATB = int_eAT @ B

        eAT = np.array(eAT).astype(np.float64)

        int_eATB = np.array(int_eATB).astype(np.float64)
        C = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 1]])


        # print(f"A: \n {A}")
        # print(f"eAT: \n {eAT}")
        # print(f"int_eAT: \n {int_eAT}")

        # print(f"B: \n {B}")
        # print(f"int_eATB: \n {int_eATB}")
        # print(f"C: \n {C}")
        self.eAT = eAT
        self.int_eATB = int_eATB
        self.int_eAT = int_eAT
        self.C = self.observer_matrix()
        self.jac_inv = np.linalg.inv(self.getyorai_gJac_linear_predict()) #Calculate Inverse Jacobian of Linearized Model Matrices

# ~~ The following functions are reference trajectories for tracking ~~
    def hover_ref_func(self, num): #Returns Constant Hover Reference Trajectories At A Few Different Positions for Testing ([x,y,z,yaw])
        hover_dict = {
            1: np.array([[0.0, 0.0, -0.5, 0.0]]).T,
            2: np.array([[0.0, 1.5, -1.5, 0.0]]).T,
            3: np.array([[1.5, 0.0, -1.5, 0.0]]).T,
            4: np.array([[1.5, 1.5, -1.5, 0.0]]).T,
            5: np.array([[0.0, 0.0, -10.0, 0.0]]).T,
            6: np.array([[1.0, 1.0, -4.0, 0.0]]).T,
            7: np.array([[3.0, 4.0, -5.0, 0.0]]).T,
            8: np.array([[1.0, 1.0, -3.0, 0.0]]).T,
        }
        if num > len(hover_dict) or num < 1:
            print(f"hover1- #{num} not found")
            exit(0)
            # return np.array([[0.0, 0.0, 0.0, self.yaw]]).T

        if not self.sim:
            if num > 4:
                print("hover modes 5+ not available for hardware")
                exit(0)
                # return np.array([[0.0, 0.0, 0.0, self.yaw]]).T
            
        print(f"hover1- #{num}")
        return hover_dict.get(num)
    
    def circle_vert_ref_func(self): #Returns Circle Reference Trajectory in Vertical Plane ([x,y,z,yaw])
        print("circle_vert_ref_func")

        t = self.timefromstart + self.T_lookahead
        w=1;
        r = np.array([[0.0, .4*np.cos(w*t), -1*(.4*np.sin(w*t)+1.5), 0.0]]).T

        return r
    
    def circle_horz_ref_func(self): #Returns Circle Reference Trajectory in Horizontal Plane ([x,y,z,yaw])
        print("circle_horz_ref_func")

        t = self.timefromstart + self.T_lookahead
        w=1;
        r = np.array([[.8*np.cos(w*t), .8*np.sin(w*t), -1.25, 0.0]]).T

        return r
    
    def fig8_horz_ref_func(self): #Returns Figure 8 Reference Trajectory in Horizontal Plane ([x,y,z,yaw])
        print("fig8_horz_ref_func")

        t = self.timefromstart + self.T_lookahead
        r = np.array([[.35*np.sin(2*t/2), .35*np.sin(t/2), -1.25, 0.0]]).T

        return r
    
    def fig8_vert_ref_func_short(self): #Returns A Short Figure 8 Reference Trajectory in Vertical Plane ([x,y,z,yaw])
        print(f"fig8_vert_ref_func_short")

        t = self.timefromstart + self.T_lookahead
        r = np.array([[0.0, .4*np.sin(t), -1*(.4*np.sin(2*t) + 1.25), 0.0]]).T

        return r
    
    def fig8_vert_ref_func_tall(self): #Returns A Tall Figure 8 Reference Trajectory in Vertical Plane ([x,y,z,yaw])
        print(f"fig8_vert_ref_func_tall")

        t = self.timefromstart + self.T_lookahead
        r = np.array([[0.0, .4*np.sin(2*t), -1*(.4*np.sin(t)+1.25), 0.0]]).T

        return r


# Entry point of the code -> Initializes the node and spins it
def main(args=None) -> None:
    print(f'Initializing "{__name__}" node for offboard control')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
    try:
        main()
    except Exception as e:
        print(e)
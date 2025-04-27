# Import libraries
import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# Import custom quadcopter and controller classes
from Quadcopter import Quadcopter
from PID_Controller import PID_Controller

#sim run time
sim_start = 0 #start time of simulation
sim_end = 15 #end time of simulation in sec
dt = 0.01 #step size in sec
time_index = np.arange(sim_start, sim_end + dt, dt)

# Initial conditions
r_ref = np.array([0., 0., 3.]) # desired position [x, y, z] in inertial frame - meters

#initial conditions
pos = [0.5, -0.5, 0.] # starting location [x, y, z] in inertial frame - meters
vel = np.array([0., 0., 0.]) #initial velocity [x; y; z] in inertial frame - m/s
ang = np.array([0., 0., 0.]) #initial Euler angles [phi, theta, psi] relative to inertial frame in deg

# Add initial random roll, pitch, and yaw rates
deviation = 10 # magnitude of initial perturbation in deg/s
random_set = np.array([random.random(), random.random(), random.random()])
ang_vel = np.deg2rad(2* deviation * random_set - deviation) #initial angular velocity [phi_dot, theta_dot, psi_dot]

ang_vel_init = ang_vel.copy()  #record for later display

gravity = 9.8 # acceleration due to gravity, m/s^2

# Gains for position controller
Kp_pos = [.95, .95, 15.] # proportional [x,y,z]
Kd_pos = [1.8, 1.8, 15.]  # derivative [x,y,z]
Ki_pos = [0.2, 0.2, 1.0] # integral [x,y,z]
Ki_sat_pos = 1.1*np.ones(3)  # saturation for integral controller (prevent windup) [x,y,z]

# Gains for angle controller
Kp_ang = [6.9, 6.9, 25.] # proportional [x,y,z]
Kd_ang = [3.7, 3.7, 9.]  # derivative [x,y,z]
Ki_ang = [0.1, 0.1, 0.1] # integral [x,y,z]
Ki_sat_ang = 0.1*np.ones(3)  # saturation for integral controller (prevent windup) [x,y,z]

# Create quadcopter with position and angle controller objects
quadcopter = Quadcopter(pos,vel,ang,ang_vel,r_ref, dt)
pos_controller = PID_Controller(Kp_pos, Kd_pos, Ki_pos, Ki_sat_pos, dt)
angle_controller = PID_Controller(Kp_ang, Kd_ang, Ki_ang, Ki_sat_ang, dt)

# Initialize results arrays
total_error = []
position_total = []
total_thrust = []

def initialize_results(res_array, num):
    for i in range(num):
        res_array.append([])

position = []
initialize_results(position,3)

velocity = []
initialize_results(velocity,3)

angle = []
initialize_results(angle,3)

angle_vel = []
initialize_results(angle_vel,3)

motor_thrust = []
initialize_results(motor_thrust,4)

body_torque = []
initialize_results(body_torque,3)



# Tether-related parameters
tether_max_length = 10.0  # maximum length of the tether in meters
init_min_height = 2.0  # minimum height of the initial height of quadcopter in meters
winding_speed = 0.05  # speed of the tether winding in m/s
force_apply_time = 0.02  # duration of the force application in seconds
force_apply_magnitude = quadcopter.mass * winding_speed / force_apply_time  # force applied to the quadcopter due to tether winding in N

def initialize_landing_anchor_position():
    ''' 
    This function initializes the position of the landing anchor. 
    The landing anchor is the point on the ground that is tethered to the quadcopter.
    '''
    while True:
        x = np.random.uniform(-tether_max_length, tether_max_length)
        y = np.random.uniform(-tether_max_length, tether_max_length)
        z = np.random.uniform(-tether_max_length, -init_min_height)  # Ensure that the quadcopter is higher than the minimum height
        #Ensure that the distance between the anchor point and the initial position of the quadcopter is within 0.5 maximum distance
        p = [x, y, z]
        if x**2 + y**2 + z**2 <= tether_max_length**2:
            return p

landing_pos_ref = initialize_landing_anchor_position() # landing anchor position [x, y, z] in inertial frame - meters



# Simulation
for idx, time in enumerate(time_index):
    print("Time: ", time)



# Write random values to screen
def write_init_ang_vel_to_screen():
    ''' 
    The program initializes with a random perturbation in angular velocity on the vehicle. 
    This simulates a wind disturbace.
    This is a display of the random disturbance
    '''
    print('Initial angular velocities (deg/s):')
    print(np.rad2deg(ang_vel_init))
    print('Total magnitude of angular velocity (deg/s)')
    print(np.linalg.norm(np.rad2deg(ang_vel_init)))

# Visualising the results
def error_plot():
    ''' Plots to the magnitude of the position error vector (m)'''
    plt.plot(time_index, total_error)
    plt.title('Quadcopter distance from reference point over time')
    plt.xlabel('time (s)')
    plt.ylabel('error (m)')
    plt.show()

def simple_plot():
    ''' 
    Plots the laterial position, vertical position, and Euler angles over time.
    This is a quick plot for trouble shooting
    '''
    fig =  plt.figure(num=None, figsize=(10, 6), dpi=80, facecolor='w', edgecolor='k')
    # Lateral position plots
    axes = fig.add_subplot(1, 3, 1)
    #axes.plot(time_index, position[0], label= 'x')
    #axes.plot(time_index, position[1], label= 'y')
    axes.plot(position[0], position[1])
    axes.set_title('Lateral Postion Over Time')
    axes.set_xlabel('x-position (m)')
    axes.set_ylabel('y-position (m)')
    axes.legend()

    # Vertical position plot
    axes = fig.add_subplot(1, 3, 2)
    axes.plot(time_index, position[2], label= 'z')
    axes.set_title('Vertical Position Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('altitude (m)')

    # Angles over time
    axes = fig.add_subplot(1, 3, 3)
    axes.plot(time_index, angle[0], label= 'phi')
    axes.plot(time_index, angle[1], label= 'theta')
    axes.plot(time_index, angle[2], label= 'psi')
    axes.set_title('Euler Angles Over Time')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('angle (deg)')
    axes.legend()

    plt.tight_layout(pad=0.4, w_pad=2.5, h_pad=2.0)
    plt.show()


def total_plot():
    '''
    This is a full plot of the results. It will plot the 3D flight path, vertical and lateral positions,
    lateral velocity, motor thrusts, body torques, Euler angles, and angular velocity of the vehicle.
    '''

    fig =  plt.figure(num=None, figsize=(16, 8), dpi=80, facecolor='w', edgecolor='k')

    # 3D Flight path
    axes = fig.add_subplot(2, 4, 1, projection='3d')
    axes.plot(position[0], position[1], position[2])
    axes.set_title('Flight Path')
    axes.set_xlabel('x (m)')
    axes.set_ylabel('y (m)')
    axes.set_zlabel('z (m)')

    # Lateral position plots
    axes = fig.add_subplot(2, 4, 2)
    axes.plot(time_index, position[0], label= 'x')
    axes.plot(time_index, position[1], label= 'y')
    axes.set_title('Lateral Postion')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('position (m)')
    axes.legend()

    # Vertical position plot
    axes = fig.add_subplot(2, 4, 3)
    axes.plot(time_index, position[2], label= 'z')
    axes.set_title('Vertical Position')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('altitude (m)')

    # Lateral velocity plots
    axes = fig.add_subplot(2, 4, 4)
    axes.plot(time_index, velocity[0], label= 'd(x)/dt')
    axes.plot(time_index, velocity[1], label= 'd(y)/dt')
    axes.plot(time_index, velocity[2], label= 'd(z)/dt')
    axes.set_title('Linear Velocity')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('velocity (m/s)')
    axes.legend()

    # Motor speed plots
    axes = fig.add_subplot(2, 4, 5)
    axes.plot(time_index, motor_thrust[0], label= 'motor 1')
    axes.plot(time_index, motor_thrust[1], label= 'motor 2')
    axes.plot(time_index, motor_thrust[2], label= 'motor 3')
    axes.plot(time_index, motor_thrust[3], label= 'motor 4')
    axes.set_title('Motor Thrust')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('Motor Thrust (N)')
    axes.legend()

    # Body torque over time
    axes = fig.add_subplot(2, 4, 6)
    axes.plot(time_index, body_torque[0], label= 'x')
    axes.plot(time_index, body_torque[1], label= 'y')
    axes.plot(time_index, body_torque[2], label= 'z')
    axes.set_title('Body Torque')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('torque (n-m)')
    axes.legend()

    # Angles over time
    axes = fig.add_subplot(2, 4, 7)
    axes.plot(time_index, angle[0], label= 'phi')
    axes.plot(time_index, angle[1], label= 'theta')
    axes.plot(time_index, angle[2], label= 'psi')
    axes.set_title('Euler Angles')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('angle (deg)')
    axes.legend()

    # Angular velocity over time
    axes = fig.add_subplot(2, 4, 8)
    axes.plot(time_index, angle_vel[0], label= 'd(phi)/dt')
    axes.plot(time_index, angle_vel[1], label= 'd(theta)/dt')
    axes.plot(time_index, angle_vel[2], label= 'd(psi)/dt')
    axes.set_title('Angular Velocity')
    axes.set_xlabel('time (s)')
    axes.set_ylabel('angular velocity (deg/s)')
    axes.legend()


    plt.tight_layout(pad=0.4, w_pad=2.5, h_pad=2.0)
    plt.show()
    

write_init_ang_vel_to_screen()
#error_plot()
#simple_plot()
total_plot()

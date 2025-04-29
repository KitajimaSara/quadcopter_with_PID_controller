# Import libraries
import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# Import custom quadcopter and controller classes and other modules
from Quadcopter import Quadcopter
from PID_Controller import PID_Controller
from Line_Intersect_3D import lineIntersect3D

#sim run time
sim_start = 0 #start time of simulation
sim_end = 300 #end time of simulation in sec
dt = 0.01 #step size in sec
time_index = np.arange(sim_start, sim_end + dt, dt)

# Initial conditions
r_ref = np.array([0., 0., 0.]) # desired position [x, y, z] in inertial frame - meters

# initial conditions
pos = [0., 0., 0.] # starting location [x, y, z] in inertial frame - meters
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
tether_max_length = 5.0  # maximum length of the tether in meters
init_min_height = 2.0  # minimum height of the initial height of quadcopter in meters
min_height = 1.5  # minimum height of the quadcopter in meters
winding_speed = 0.05  # speed of the tether winding in m/s
force_apply_time = 0.02  # duration of the force application in seconds
force_apply_magnitude = quadcopter.mass * winding_speed / force_apply_time  # force applied to the quadcopter due to tether winding in N
class Tethering_Point:
    '''
    This class represents the tethering point of the quadcopter in inertial frame. 
    It is used to save the position and vector of the tethering situation.
    '''
    def __init__(self):
        self.point = []

    def add_point(self, position, vector):
        '''
        This function adds a point to the tethering point.
        '''
        self.point.append((position, vector))

    def get_point(self, idx):
        '''
        This function returns the point at the given index.
        '''
        return self.point[idx]
    
    def get_position(self, idx):
        '''
        This function returns the position of the tethering point.
        '''
        return self.point[idx][0]
    
    def get_vector(self, idx):
        '''
        This function returns the vector of the tethering point.
        '''
        return self.point[idx][1]
    
    def get_all_points(self):
        '''
        This function returns all the points of the tethering point.
        '''
        return self.point
tethering_point = Tethering_Point() # create an Tethering_Point class data to save the tethering point data

# def calculate_landing_anchor_estimate():
#     '''
#     This function calculates the estimated position of the landing anchor. 
#     The landing anchor is the point on the ground that is tethered to the quadcopter.
#     '''
#     # Get all tethering points data
#     start_point = []
#     end_point = []
#     if len(tethering_point.point) > 1:
#         # Get all points
#         for i in range(len(tethering_point.point)):
#             # Get the point and vector
#             start_point.append(tethering_point.get_point(i))
#             end_point.append(tethering_point.get_point(i) + tethering_point.get_vector(i))
#     if start_point != [] and end_point != []:
#         start_point = np.array(start_point)
#         end_point = np.array(end_point)
#         intersect_point, distances = lineIntersect3D(start_point, end_point)
#     else:
#         intersect_point = []
#     return intersect_point

def calculate_landing_anchor_estimate():
    '''
    This function calculates the estimated position of the landing anchor. 
    The landing anchor is the point on the ground that is tethered to the quadcopter.
    '''
    start_point = []
    end_point = []
    if len(tethering_point.point) > 1:
        for i in range(len(tethering_point.point)):
            start = np.array(tethering_point.get_position(i))
            vector = np.array(tethering_point.get_vector(i))
            start_point.append(start)
            end_point.append(start + vector)
        
        start_point = np.array(start_point)
        end_point = np.array(end_point)

        intersect_point, distances = lineIntersect3D(start_point, end_point)
    else:
        intersect_point = []
    return intersect_point

def generate_next_tethering_point():
    '''
    This function generates the next tethering point. 
    The tethering point is the point in the inertial frame that the quadcopter receive tethering force.
    '''
    # if the number of existing tethering point < 2, generate the point specifically
    if len(tethering_point.point) < 2:
        while True:
            # Generate a random point in the inertial frame
            prev_point = tethering_point.get_position(0) if len(tethering_point.point) > 0 else [0, 0, 0]
            x = np.random.uniform(-tether_max_length * 0.3 + prev_point[0], tether_max_length * 0.3 + prev_point[0])
            y = np.random.uniform(-tether_max_length * 0.3 + prev_point[1], tether_max_length * 0.3 + prev_point[1])
            z = np.random.uniform(prev_point[2], tether_max_length * 0.3 + prev_point[2])
            p = [x - prev_point[0], y - prev_point[1], z - prev_point[2]]
            if tether_max_length * 0.1 < np.linalg.norm(p) <= tether_max_length * 0.3:
                return [x, y, z]
    # if the number of existing tethering point >= 2, generate the point refering to the landing point
    if len(tethering_point.point) > 1:
        prev_point = tethering_point.get_position(-1)
        landing_pos_est = calculate_landing_anchor_estimate()
        while True:
            # Generate a random point in the inertial frame
            x = np.random.uniform(-tether_max_length * 0.3 + prev_point[0], tether_max_length * 0.3 + prev_point[0])
            y = np.random.uniform(-tether_max_length * 0.3 + prev_point[1], tether_max_length * 0.3 + prev_point[1])
            z = np.random.uniform(-tether_max_length * 0.3 + prev_point[2], tether_max_length * 0.3 + prev_point[2])
            p1 = [x - prev_point[0], y - prev_point[1], z - prev_point[2]]
            p2 = [x - landing_pos_est[0], y - landing_pos_est[1], z - landing_pos_est[2]]
            if tether_max_length * 0.1 < np.linalg.norm(p1) <= tether_max_length * 0.3 and np.linalg.norm(p2) <= tether_max_length * 0.9 and p2[2] > min_height:
                return [x, y, z]

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

def calculate_tether_force(force_magnitude, quadcopter_pos):
    ''' 
    This function calculates the force applied to the quadcopter due to the tether winding.
    The force is applied in the direction of the tether vector.
    '''
    # Calculate the tether vector
    tether_vector = np.array(quadcopter_pos - landing_pos_ref)
    # Calculate the length of the tether
    tether_length = np.linalg.norm(tether_vector)
    # Normalize the tether vector
    if tether_length > 0:
        unit_tether_vector = tether_vector / tether_length
        # Calculate the force vector
        force_vector = -force_magnitude * unit_tether_vector
        return force_vector
    else:
        return np.array([0.0, 0.0, 0.0])  # No force if the quadcopter is at the anchor point

apply_force_flag = False
force_timer = 0.0
force_apply_duration = force_apply_time  # 已经在文件里有定义
anchor_estimates = []
anchor_estimates_idx = []



# Simulation
for idx, time in enumerate(time_index):
    
    #find position and velocity error and call positional controller
    pos_error = quadcopter.calc_pos_error(quadcopter.pos)
    vel_error = quadcopter.calc_vel_error(quadcopter.vel)
    des_acc = pos_controller.control_update(pos_error,vel_error)

    #Modify z gain to include thrust required to hover
    des_acc[2] = (gravity + des_acc[2])/(math.cos(quadcopter.angle[0]) * math.cos(quadcopter.angle[1]))
    
    #calculate thrust needed  
    thrust_needed = quadcopter.mass * des_acc[2]

    #Check if needed acceleration is not zero. if zero, set to one to prevent divide by zero below
    mag_acc = np.linalg.norm(des_acc)
    if mag_acc == 0:
        mag_acc = 1
    
    #use desired acceleration to find desired angles since the quad can only move via changing angles
    ang_des = [math.asin(-des_acc[1] / mag_acc / math.cos(quadcopter.angle[1])),
        math.asin(des_acc[0] / mag_acc),
         0]

    #check if exceeds max angle
    mag_angle_des = np.linalg.norm(ang_des)
    if mag_angle_des > quadcopter.max_angle:
        ang_des = (ang_des / mag_angle_des) * quadcopter.max_angle

    #call angle controller
    quadcopter.angle_ref = ang_des
    ang_error = quadcopter.calc_ang_error(quadcopter.angle)
    ang_vel_error = quadcopter.calc_ang_vel_error(quadcopter.ang_vel)
    tau_needed = angle_controller.control_update(ang_error, ang_vel_error)

    #Find motor speeds needed to achieve desired linear and angular accelerations
    quadcopter.des2speeds(thrust_needed, tau_needed)

    # Step in time and update quadcopter attributes
    quadcopter.step()


    # landing point estimation
    
    # 先检查无人机是否稳定
    if quadcopter.is_stable and not apply_force_flag:
        # 记录当前稳定位置
        stable_position = quadcopter.pos.copy()

        # 根据当前位置计算需要施加的外力方向
        force_vector = calculate_tether_force(force_apply_magnitude, quadcopter.pos)

        # 施加外力
        quadcopter.apply_external_force(force_vector)

        # 开启力检测
        quadcopter.force_detector_on = True

        # 开始计时
        apply_force_flag = True
        force_timer = 0.0

    elif apply_force_flag:
        force_timer += dt

        # 到达施力时间，停止施力
        if force_timer >= force_apply_duration:
            # 取消外力
            quadcopter.apply_external_force(np.array([0., 0., 0.]))

            # 关闭力检测
            quadcopter.force_detector_on = False

            # 存储施力时检测到的外力方向
            estimated_force_vector = quadcopter.external_force_vector_estimate.copy()

            tethering_point.add_point(stable_position, estimated_force_vector)

            # 更新锚点估计
            anchor_estimate = calculate_landing_anchor_estimate()
            if len(anchor_estimate) != 0:
                anchor_estimates.append(anchor_estimate)
                anchor_estimates_idx.append(len(anchor_estimates_idx))

            # 生成下一个飞行目标点
            next_tether_point = generate_next_tethering_point()
            quadcopter.pos_ref = np.array(next_tether_point)

            # 重置状态
            apply_force_flag = False

    # Record key attributes for plotting
    position_total.append(np.linalg.norm(quadcopter.pos))

    position[0].append(quadcopter.pos[0])
    position[1].append(quadcopter.pos[1])
    position[2].append(quadcopter.pos[2])

    velocity[0].append(quadcopter.vel[0])
    velocity[1].append(quadcopter.vel[1])
    velocity[2].append(quadcopter.vel[2])

    angle[0].append(np.rad2deg(quadcopter.angle[0]))
    angle[1].append(np.rad2deg(quadcopter.angle[1]))
    angle[2].append(np.rad2deg(quadcopter.angle[2]))

    angle_vel[0].append(np.rad2deg(quadcopter.ang_vel[0]))
    angle_vel[1].append(np.rad2deg(quadcopter.ang_vel[1]))
    angle_vel[2].append(np.rad2deg(quadcopter.ang_vel[2]))

    motor_thrust[0].append(quadcopter.speeds[0]*quadcopter.kt)
    motor_thrust[1].append(quadcopter.speeds[1]*quadcopter.kt)
    motor_thrust[2].append(quadcopter.speeds[2]*quadcopter.kt)
    motor_thrust[3].append(quadcopter.speeds[3]*quadcopter.kt)

    body_torque[0].append(quadcopter.tau[0])
    body_torque[1].append(quadcopter.tau[1])
    body_torque[2].append(quadcopter.tau[2])

    total_thrust.append(quadcopter.kt * np.sum(quadcopter.speeds))

    # Positional error
    r_error = quadcopter.pos_ref - quadcopter.pos
    total_error.append(np.linalg.norm(r_error))



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

# def plot_anchor_estimates():
#     '''Plot the estimated anchor points compared with true anchor point'''
#     anchor_estimates_arr = np.array(anchor_estimates)
#     true_anchor = np.array(landing_pos_ref)

#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     if anchor_estimates_arr.shape[0] > 0:
#         ax.scatter(anchor_estimates_arr[:,0], anchor_estimates_arr[:,1], anchor_estimates_arr[:,2], c='b', label='Estimated Anchors')

#     ax.scatter(true_anchor[0], true_anchor[1], true_anchor[2], c='r', label='True Anchor', marker='^', s=100)

#     for idx, (x, y, z) in enumerate(anchor_estimates_arr):
#         ax.text(x, y, z, f'{idx}', size=8, zorder=1, color='k')  # 标上编号

#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#     ax.set_title('Estimated vs True Landing Anchor Points')
#     ax.legend()
#     plt.show()

# # 仿真结束后调用
# plot_anchor_estimates()

def plot_results_combined():
    '''Plot the results: anchor estimates, flight path, error over time'''
    anchor_estimates_arr = np.array(anchor_estimates)
    true_anchor = np.array(landing_pos_ref)

    fig = plt.figure(figsize=(14,10))

    # 3D飞行轨迹
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(position[0], position[1], position[2], label='Flight Path')
    ax1.scatter(true_anchor[0], true_anchor[1], true_anchor[2], c='r', marker='^', s=80, label='True Anchor')
    ax1.set_title('3D Flight Path')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.legend()

    # 锚点估计散点图
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    if anchor_estimates_arr.shape[0] > 0:
        ax2.scatter(anchor_estimates_arr[:,0], anchor_estimates_arr[:,1], anchor_estimates_arr[:,2], c='b', label='Estimated Anchors')
    ax2.scatter(true_anchor[0], true_anchor[1], true_anchor[2], c='r', marker='^', s=80, label='True Anchor')

    for idx, (x, y, z) in enumerate(anchor_estimates_arr):
        ax2.text(x, y, z, f'{idx}', size=8, color='k')  # 编号标记

    ax2.set_title('Anchor Estimates')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.legend()

    # 锚点估计误差变化曲线
    ax3 = fig.add_subplot(2, 2, 3)
    error_list = []
    for est in anchor_estimates:
        err = np.linalg.norm(est - true_anchor)
        error_list.append(err)
    
    if len(error_list) > 0:
        ax3.plot(anchor_estimates_idx, error_list, marker='o')
        ax3.set_title('Anchor Estimate Error Over Iterations')
        ax3.set_xlabel('Estimation Step')
        ax3.set_ylabel('Estimation Error (m)')
        ax3.grid(True)

    # 飞行高度变化
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(time_index, position[2])
    ax4.set_title('Altitude Over Time')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Altitude (m)')
    ax4.grid(True)

    plt.tight_layout()
    plt.show()

plot_results_combined()
#error_plot()
#simple_plot()
total_plot()

import numpy as np
import math

class Waypoint:
    def __init__(self, x, y, z, elevation, azimuth, velocity):
        self.x = x
        self.y = y
        self.z = z
        self.elevation = elevation
        self.azimuth = azimuth
        self.velocity = velocity

class Trajectory:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint = 0
        self.completed = False

    def advance_waypoint(self):
        self.current_waypoint += 1
        if self.current_waypoint >= len(self.waypoints):
            self.completed = True
            self.current_waypoint = -1

    def __getitem__(self, key):
        return self.waypoints[key]

    



pose = np.array([0., 0., 0., 0., 0., 0.]) # x, y, z, yaw, pitch, roll
velocity = np.array([0., 0., 0., 0., 0., 0.])
accel = np.array([0., 0., 0., 0., 0., 0.])


traj = Trajectory([Waypoint(0., 0., 0., 0., 0., 0.), Waypoint(1., 1., 1., 0., 0., 0.)])
mass = 1.



'''
@param V_v: velocity of the vehicle
@param V_d: desired velocity
@param k_v: gain of the velocity controller
@param theta_v: elevation angle of the vehicle
@param psi_v: azimuth angle of the vehicle
@param theta_dot_v: rate of pitch of the vehicle
@param psi_dot_v: yaw rate of the vehicle
'''
def get_acceleration_vector(V_v, V_d, k_v, theta_v, psi_v, theta_dot_v, psi_dot_v) :
    c_theta = np.cos(theta_v)
    s_theta = np.sin(theta_v)
    c_psi = np.cos(psi_v)
    s_psi = np.sin(psi_v)

    V_v_dot = k_v * (V_d - V_v)

    x_accel_v = V_v_dot * c_theta * c_psi - theta_dot_v * V_v * s_theta * c_psi - psi_dot_v * V_v * c_theta * s_psi
    y_accel_v = V_v_dot * c_theta * s_psi - theta_dot_v * V_v * s_theta * s_psi + psi_dot_v * V_v * c_theta * c_psi
    z_accel_v = -theta_dot_v * V_v * c_theta - V_v_dot * s_theta

    return np.array([x_accel_v, y_accel_v, z_accel_v - 32.])

'''
@param accel_vector: acceleration vector of the vehicle
@param m: mass of the vehicle
'''
def get_thrust(accel_vector, m):
    return m * np.sqrt(accel_vector.T @ accel_vector)

'''
@param yaw: yaw angle of the vehicle
@param pitch: pitch angle of the vehicle
@param roll: roll angle of the vehicle
@param acceleration_vector: acceleration vector of the vehicle
@param m: mass of the vehicle
@param T: thrust of the vehicle
'''
def get_desired_angles(yaw, pitch, roll, acceleration_vector, m, T):
    c_yaw = np.cos(yaw)
    s_yaw = np.sin(yaw)
    c_pitch = np.cos(pitch)
    s_pitch = np.sin(pitch)
    c_roll = np.cos(roll)
    s_roll = np.sin(roll)

    R_yaw = np.array([[c_yaw, -s_yaw, 0.], [s_yaw, c_yaw, 0.], [0., 0., 1.]])
    R_pitch = np.array([[c_pitch, 0., s_pitch], [0., 1., 0.], [-s_pitch, 0., c_pitch]])
    R_roll = np.array([[1., 0., 0.], [0., c_roll, -s_roll], [0., s_roll, c_roll]])

    # Z = R_pitch.T @ R_roll.T @ np.array([[0.], [0.], [1.]]) TODO: check this
    Z = R_yaw @ acceleration_vector * (m / -T)
    
    phi_d = np.arcsin(-Z[1])
    theta_d = np.arctan2(Z[0], Z[2])

    return np.array([phi_d, theta_d])
    
def has_crossed_waypoint(pose, waypoint, tolerance):
    return abs(pose[0] - waypoint.x) < tolerance and abs(pose[1] - waypoint.y) < tolerance and abs(pose[2] - waypoint.z) < tolerance

def output_to_arduino(accel_vector, thrust):
    print("Accel: " + str(accel_vector))
    print("Thrust: " + str(thrust))
    print("")

while(True):
    if traj.completed:
        break
    if has_crossed_waypoint(pose, traj.waypoints[traj.current_waypoint], 0.1):
        traj.advance_waypoint()

    accel_vector = get_acceleration_vector(velocity.dot(velocity), traj.waypoints[traj.current_waypoint].velocity, 1., traj[traj.current_waypoint].elevation, traj[traj.current_waypoint].azimuth, accel[4], accel[5])
    thrust = get_thrust(accel_vector, mass)
    desired_angles = get_desired_angles(pose[3], pose[4], pose[5], accel_vector, mass, thrust)

    output_to_arduino(desired_angles, thrust)

# print(get_thrust(get_acceleration_vector(2, 10, 0.4, 0.0, 0.2, 0, 0), 0.8))
# print(get_desired_angles(0.0, 0.0, 0.0, get_acceleration_vector(2, 10, 0.4, 0.0, 0.2, 0, 0), 0.8, 0.8))
# Pure-Pursuit
This repository contains the code to follow trajectories

## Basic Theory
The Pure Pursuit Algorithm is an extremely common and reliable algorithm for following paths.
1. Define a trajectory of waypoints
2. Calculate the drone's current position, using the Odometry
3. Figure out target outputs which will be an elevation angle and an azimuth angle
4. Convert the elevation and azimuth angle to yaw, pitch, roll using acceleration vectors and rotation matrices
5. Output this to the flight controller

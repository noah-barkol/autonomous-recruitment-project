import numpy as np
from simulator import Simulator, centerline

sim = Simulator()

def controller(x):
    """controller for a car

    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]

    Returns:
        ndarray: numpy array of shape (2,) containing [fwd acceleration, steering rate]
    """
    # Fixed values, referenced later
    WHEELBASE = 1.58                # distance between front and rear wheels
    MAX_ACCELERATION = 12           # m/s^2, in both x and y combined
    TIME_STEP = 0.01                # timestamp between calls in the run function of simulator
    
    # Feedback inputs below, are noisy in real system with state estimation required
    xpos   = x[0]                   # current x position, first index of numpy array
    ypos   = x[1]                   # current y position
    phi    = np.mod(x[2], 2*np.pi)  # current heading (radians), gives most reduced version of angle
    v      = x[3]                   # current velocity
    theta   = x[4]                  # current steering angle (sim.steering_limits: -0.7 to 0.7)
    
    # Dividing centerline into 1000 increments
    indices = 1000
    increments = np.linspace(0, 105, indices)       # creates indices along centerline
    meters_per_index = 105 / indices                # static variable representing amount of meters in each increment
    points = centerline(increments)                 # gets x,y coordinates for each index

    # Puts input values of current position into an array for numpy to use 
    current_position = np.array([xpos, ypos])        
    
    # Finds distance to points on centerline using numpy
    distances = np.sqrt((points[:, 0] - current_position[0])**2 + (points[:, 1] - current_position[1])**2)   
    
    # Finds index of closest coordinate pair by finding minimum of all the distances from the current position to all the points
    closest_index = np.argmin(distances)  

    # Calculate how many indices you'll pass with current speed
    meters = max(v * TIME_STEP * 5, 1.0)  # *** might need to be multiplied by a gain variable to amplify time_step
    lookahead = meters / meters_per_index

    # Find next index of arc_lengths and thus next position to travel to
    next_index = min(closest_index + int(lookahead), len(points) - 1)   # second argument accounts for end of track
    next_position = points[next_index]                                  # searches in points for an x,y coordinate for next_index
    next_x = next_position[0]
    next_y = next_position[1]
    
    # Find difference of header direction from angle to the next point
    angle_to_point = np.arctan2(next_y - ypos, next_x - xpos)                                     # arctan2 takes x and y separately
    heading_difference = np.arctan2(np.sin(angle_to_point - phi), np.cos(angle_to_point - phi))   # wraparound to avoid overcorrection
    theta_desired = np.clip(heading_difference, -0.7, 0.7)                                        # steering angle must be between -0.7 and 0.7
    next_steer_angle = theta_desired - theta   # sees how far the current steering angle is away from the target, its bounds are -1.4 to 1.4

    # Finds steering velocity by dividing steer angle needed by amount of time, then making sure it's within the constraints
    theta_dt = next_steer_angle / TIME_STEP
    theta_dt = np.clip(theta_dt, sim.lbu[1], sim.ubu[1])   # clip puts theta_dt in constraints of -1.0 to 1.0 for simulator.py
    
    # Ensure acceleration output is safe with current conditions
    lateral_accel = (v**2 / (WHEELBASE / 2)) * np.sin(np.arctan(0.5 * np.tan(theta))) # from _get_accel in simulator.py
    accel_left = np.sqrt(max(MAX_ACCELERATION**2 - lateral_accel**2, 0)) # calculates how high acceleration output can go
    
    # PLACEHOLDER VALUES FOR VELOCITY AND NEXT_STEER_ANGLE. Acceleration range: (sim.accel_limits: -10 to 4)
    if abs(theta_desired) > 0.5:            # theta_desired is how much front wheels are about to be turned
        if v > 7.0:                         # large steer angle and high velocity = decelerate
            accel_output = -6.0
        else:
            accel_output = -1.0
    elif abs(theta_desired) > 0.4:
        if v > 7.0:
            accel_output = -2.0
        else:
            accel_output = 0.5
    elif v < 5.0:
        accel_output = 3.0
    elif v > 12.0:
        accel_output = -1.0
    elif v > 7.0:
        accel_output = 1.0
    else:
        accel_output = 2.0
        
    accel_output = np.clip(accel_output, -accel_left, accel_left)
    accel_output = np.clip(accel_output, sim.lbu[0], sim.ubu[0])
    
    return np.array([accel_output, theta_dt])        




sim.set_controller(controller)
sim.run()

ts, xs, us, crash, slip = sim.get_results()
print("any crash:", np.any(crash))
print("any slip:", np.any(slip))
print("crash count:", np.sum(crash))
print("slip count:", np.sum(slip))
print("max v:", np.max(xs[3]))
print("final position:", xs[0, -1], xs[1, -1])

# sim.animate()   # commented out -- no display in Codespaces
# sim.plot()

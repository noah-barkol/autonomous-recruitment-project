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
    dt = 0.01                       # timestamp between calls in the run function of simulator.py
    
    # Feedback inputs below, are noisy in real system with state estimation required
    xpos   = x[0]                   # current x position, first index of numpy array
    ypos   = x[1]                   # current y position
    phi    = np.mod(x[2], 2*np.pi)  # current heading (radians), gives most reduced version of angle
    v      = x[3]                   # current velocity
    theta   = x[4]                  # current steering angle (sim.steering_limits: -0.7 to 0.7)
    
    increments = np.linspace(0, 105, 1000)          # what to call centerline on; creates 1000 increments along centerline
               
    points = centerline(increments)                 # gets x,y coordinates for each increment

    current_position = np.array([xpos, ypos])       # puts input values of current position into an array for numpy to use  
    
    # Finds distance to points on centerline using numpy
    distances = np.sqrt((points[:, 0] - current_position[0])**2 + (points[:, 1] - current_position[1])**2)   
    
    # Finds index of closest coordinate pair by finding minimum of all the distances from the current position to all the points
    closest_index = np.argmin(distances)     # switched from np.min to np.argmin to get index instead of distance value


    # Find next index of arc_lengths and thus next position to travel to
    next_index = min(closest_index + 5, 1000)     # uses 5 as PLACEHOLDER VALUE to jump across points   
    next_position = points[next_index]            # searches in points for an x,y coordinate for next_index
    next_x = next_position[0]
    next_y = next_position[1]
    
    # Find difference of header direction from angle to the next point
    angle_to_point = np.arctan2(next_y - ypos, next_x - xpos) # arctan2 takes x and y separately
    heading_difference = angle_to_point - phi

    theta_desired = np.clip(heading_difference, -0.7, 0.7)  # steering angle must be between -0.7 and 0.7
    next_steer_angle = theta_desired - theta                # sees how far the current steering angle is away from the target

    # Created a variable that stands in for dividing next_steer_angle by dt. I want this variable is proportional to velocity. This would make the steering velocity higher when the car is faster and needs more force to adjust
    speed_steer = 2.0 #PLACEHOLDER VALUE
    
    theta_dt = next_steer_angle * speed_steer
    theta_dt = np.clip(theta_dt, sim.lbu[1], sim.ubu[1])   # clip puts theta_dt in constraints of -1.0 to 1.0 for simulator.py
    
    # Ensure acceleration output is safe with current conditions. Don't use too much energy
    lateral_accel = (v**2 / (WHEELBASE / 2)) * np.sin(np.arctan(0.5 * np.tan(theta))) # from _get_accel in simulator.py
    juice_left = np.sqrt(max(MAX_ACCELERATION - lateral_accel, 0))
    
    # PLACEHOLDER VALUES FOR VELOCITY AND NEXT_STEER_ANGLE. acceleration range: (sim.accel_limits: -10 to 4)
    if abs(next_steer_angle) > 0.3: # large steer angles required = decelerate
        accel_output = -6.0
    elif abs(next_steer_angle) > 0.1:
        accel_output = -2.0
    elif v < 5.0:
        accel_output = 3.0
    elif v > 10.0:
        accel_output = -1.0
    else:
        accel_output = 0.0
        
    accel_output = np.clip(accel_output, -juice_left, juice_left)
    accel_output = np.clip(accel_output, sim.lbu[0], sim.ubu[0])
    return np.array([accel_output, theta_dt])        




sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()

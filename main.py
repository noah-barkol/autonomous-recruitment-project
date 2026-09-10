import numpy as np
from simulator import Simulator, centerline

sim = Simulator()

arc_length = 0.0                   # starting point for arc_length, will increment in controller method

def controller(x):
    """controller for a car

    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]

    Returns:
        ndarray: numpy array of shape (2,) containing [fwd acceleration, steering rate]
    """
    WHEELBASE = 1.58                # distance between front and rear wheels
    MAX_ACCELERATION = 12           # m/s^2, in both x and y combined
    dt = 0.01                       # timestamp between calls in the run function of simulator.py
    increment_distance = 
    
    # Feedback inputs below, are noisy in real system with state estimation required
    xpos   = x[0]                   # current x position, first index of numpy array
    ypos   = x[1]                   # current y position
    phi    = np.mod(x[2], 2*np.pi)  # current heading (radians), gives most reduced version of angle
    v      = x[3]                   # current velocity
    theta   = x[4]                  # current steering angle (sim.steering_limits: -0.7 to 0.7)
    acc_output = 0
    theta_dt = 0
    
    increments = np.linspace(0, 105, 1000)          # what to call centerline on; creates 1000 increments along centerline
               
    points = centerline(increments)                 # gets x,y coordinates for each increment

    # Find closest point on centerline to car's current position
    current_position = np.array([xpos, ypos])       # puts input values of current position into an array for numpy to use  
    # Finds arc length (defined as s by Jaden) to points on centerline using numpy
    arc_lengths = np.sqrt((points[:, 0] - current_position[0])**2 + (points[:, 1] - current_position[1])**2)   # researched that : traverses through columns
    # Finds index of closest coordinate pair by finding minimum of all the distances from the current position to all the points
    closest_index = np.argmin(arc_lengths)     # switched from np.min to np.argmin to get index instead of distance value


    # Find next index of arc_lengths and thus next position to travel to
    next_index = closest_index + 10               # using 10 as a placeholder. not sure what to do at end of track    
    next_position = points(next_index)            # searches in points for an x,y coordinate for next_index
    next_x = next_position[0]
    next_y = next_position[1]

    # Use current heading and steering direction to know where pointed and going respectively, then adjust steering direction at a certain intensity.
        # For this, I need to know how far off the heading/direction are, so I need to use 
    #abs value
    angle_to_point = arctan(abs((next_y - ypos)/(next_x - xpos)))
    angle_phi = phi * np.pi / 180            # current heading in angles instead of radians

    # The higher the theta_dt value is (how intensely the car is going to be turned), acceleration should be lower
    
    return np.array([acc_output, theta_dt])          # acceleration range: (sim.accel_limits: -10 to 4) and theta dt range: (-1.0 to 1.0)




sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()

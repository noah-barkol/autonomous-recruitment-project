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
    
    line_coordinates = centerline(np.linspace([0, 105, 1000])
               
    points = centerline(line_coordinates)          # 1000 checkpoints to 
    next_position = (arc_length + increment_distance)   # returns (x,y) coordinate, can use with x,y coordinates of left and right cones to figure out how fast to change angle
    next_x = next_position[0]
    next_y = next_position[1]

    return np.array([acc_output, theta_dt])          # acceleration range: (sim.accel_limits: -10 to 4) and theta dt range: (-1.0 to 1.0)




sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()

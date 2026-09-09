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
    
    # Feedback inputs below, are noisy in real system with state estimation required
    xpos   = x[0]                   # current x position, first index of numpy array
    ypos   = x[1]                   # current y position
    phi    = np.mod(x[2], 2*np.pi)  # current heading (radians), gives most reduced version of angle
    v      = x[3]                   # current velocity
    theta   = x[4]                  # current steering angle (-0.7 to 0.7)

    arc_length += v * dt           # increment total distance covered using v = x/t (assuming constant acceleration during short interval)
    
    line = sim.centerline(arc_length)  
    
    return np.array([0,0])          # acceleration range: (-10 to 4) and theta dt range: (-1.0 to 1.0)




sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()

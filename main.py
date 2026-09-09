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
    xpos   = x[0]                   # current x position, first index of numpy array
    ypos   = x[1]                   # current y position
    phi    = np.mod(x[2], 2*np.pi)  # current heading (radians), gives most reduced version of angle
    v      = x[3]                   # current velocity
    theta   = x[4]                  # current steering angle (-0.7 to 0.7)

    # consider feeback is noisy
    
    

    return np.array([0,0])          # return recommended acceleration (range: -10 to 4) and time derivative of steering angle (range: -1.0 to 1.0)




sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()

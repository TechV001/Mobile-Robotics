from grid import *
from particle import Particle
from utils import *
from setting import *

# ------------------------------------------------------------------------
def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- noisy odometry measurements, a pair of robot pose, i.e. last time
                step pose and current time step pose

        Returns: the list of particle representing belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    alpha1 = 0.001
    alpha2 = 0.001
    alpha3 = 0.005
    alpha4 = 0.005

    
    return particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- a list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
        grid -- grid world map containing the marker information. 
                see grid.py and CozGrid for definition

        Returns: the list of particle representing belief p(x_{t} | u_{t})
                after measurement update
    """
    measured_particles = []
    return measured_particles

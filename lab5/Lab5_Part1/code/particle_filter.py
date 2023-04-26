from grid import *
from particle import Particle
from utils import *
from setting import *
import math
import numpy as np

# ------------------------------------------------------------------------
def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- noisy odometry measurements, a pair of robot pose, i.e. last time
                step pose and current time step pose. odom[0] is prev and odom[1] is current

        Returns: the list of particle representing belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    alpha1 = 0.001
    alpha2 = 0.001
    alpha3 = 0.005
    alpha4 = 0.005

    updated_particles = []
    for particle in particles:
        # Calculate the motion model based on the odometry measurements
        
        
        delta_rot1 = math.atan2(odom[1][1], odom[1][0])
        delta_trans = grid_distance(odom[0][0], odom[0][1], odom[1][0], odom[1][1])
        delta_rot2 = odom[1][2] - delta_rot1
        
        # Add noise to the motion model
        delta_rot1_hat = delta_rot1 - 0.001 * random.gauss(0, alpha1 * delta_rot1 + alpha2 * delta_trans)
        delta_trans_hat = delta_trans - 0.001 * random.gauss(0, alpha3 * delta_trans + alpha4 * (delta_rot1 + delta_rot2))
        delta_rot2_hat = delta_rot2 - 0.001 * random.gauss(0, alpha1 * delta_rot2 + alpha2 * delta_trans)
        
        # Update the particle's pose based on the noisy motion model
        new_x = particle.x + delta_trans_hat * math.cos(particle.h + delta_rot1_hat)
        new_y = particle.y + delta_trans_hat * math.sin(particle.h + delta_rot1_hat)
        new_h = particle.h + delta_rot1_hat + delta_rot2_hat
        
        # Add the updated particle to the list of updated particles
        updated_particles.append(Particle(new_x, new_y, new_h))
    
    return updated_particles

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
    measured_weights = []
    
    for particle in particles:
        weight = 1.0
        #nearest_marker = None
        #nearest_dist = float("inf")
        for measured_marker in measured_marker_list:
            rx, ry, rh = measured_marker

            # Transform the measured marker from the robot's frame to the world frame
            
            if measured_marker is not None:
                # Calculate the likelihood of the measured marker given the particle's pose
                distance_error = grid_distance(measured_marker[0], measured_marker[1], rx, ry)
                heading_error = diff_heading_deg(measured_marker[2], rh)
                phi_hat = math.atan2(measured_marker[1] - ry, measured_marker[0] - rx) - rh
                
                # Update the particle's weight
                weight *= 1/(math.sqrt(2*math.pi*(MARKER_TRANS_SIGMA**2))) * (math.e ** (-0.5 * (distance_error**2 / (0.1**2) + heading_error**2 / (MARKER_TRANS_SIGMA**2))))
                
        # Set the particle's weight and add it to the list of measured particles
        measured_weights.append(weight)
        measured_particles.append(particle)

    # Normalize the particle weights
    i = 0
    total_weight = sum(measured_weights)
    normalized_weights = []
    
    if(total_weight != 0):
      for i in measured_weights:
         new_weight = i / total_weight
         normalized_weights.append(new_weight)
         
      if(len(measured_weights) != 0):
         measured_particles = np.random.choice(particles, PARTICLE_COUNT-5, replace = True, p = normalized_weights)
    
    else:
      return particle.create_random(PARTICLE_COUNT, grid)
    return measured_particles

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
        
        
        delta_rot1 = math.degrees(math.atan2(odom[1][1] - odom[0][1], odom[1][0] - odom[0][0])) - proj_angle_deg(odom[0][2])
        delta_trans = grid_distance(odom[0][0], odom[0][1], odom[1][0], odom[1][1])
        delta_rot2 = diff_heading_deg(diff_heading_deg(odom[1][2],odom[0][2]), delta_rot1)
        
        # Add noise to the motion model
        delta_rot1_hat = delta_rot1 - random.gauss(0, alpha1 * delta_rot1 + alpha2 * delta_trans)
        delta_trans_hat = delta_trans - random.gauss(0, alpha3 * delta_trans + alpha4 * (delta_rot1 + delta_rot2))
        delta_rot2_hat = delta_rot2 - random.gauss(0, alpha1 * delta_rot2 + alpha2 * delta_trans)
        
        # Update the particle's pose based on the noisy motion model
        new_x = particle.x + delta_trans_hat * math.cos(math.radians(particle.h) + math.radians(delta_rot1_hat))
        new_y = particle.y + delta_trans_hat * math.sin(math.radians(particle.h) + math.radians(delta_rot1_hat))
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
    weights = []
    
    for particle in particles:
        pair = []
        p_marker = particle.read_markers(grid)
        px,py,ph = particle.xyh
        
        for measured_marker in measured_marker_list:
            rx, ry, rh = add_marker_measurement_noise(measured_marker, MARKER_TRANS_SIGMA, MARKER_ROT_SIGMA)
            
            if(p_marker is not None):
               nearest_marker = None
               min_dist = float("inf")
               
               for marker in p_marker:
                  dist = grid_distance(rx,ry,marker[0],marker[1])
                  if(dist < min_dist):
                     min_dist = dist
                     nearest_marker = marker
            pair.append((measured_marker, nearest_marker))
        if(grid.is_in(px,py) == False):
            weights.append(0)
        elif(len(p_marker) == 0 and len(measured_marker_list) == 0):
            weights.append(1)
        elif(len(p_marker) != len(measured_marker_list)):
            weights.append(0)
        elif(len(pair) != 0):
            p = 1
            for i, j in pair:
               r_hat = grid_distance(p_marker[0][0],p_marker[0][1],i[0],i[1])#grid_distance(p_marker[0][0],p_marker[0][1],px,py)
               phi_hat = diff_heading_deg(i[2],p_marker[0][2])
               r_range = math.sqrt(j[0]**2 + j[1]**2)
               phi = proj_angle_deg(j[2])
               #r_range = grid_distance(j[0],j[1],i[0],i[1])
               #phi = diff_heading_deg(j[2],p_marker[0][2])
               r_hat = (-0.5*((r_range-r_hat)**2/MARKER_TRANS_SIGMA**2))
               phi_hat = (-0.5*((phi-phi_hat)**2/MARKER_ROT_SIGMA**2))
               #r_hat = (1/(math.sqrt(2*math.pi)*MARKER_TRANS_SIGMA)) * (math.e ** (-0.5*((r_range-r_hat)**2/MARKER_TRANS_SIGMA**2)))
               #phi_hat = (1/(math.sqrt(2*math.pi)*MARKER_ROT_SIGMA)) * (math.e ** (-0.5*((phi-phi_hat)**2/MARKER_ROT_SIGMA**2)))
               p *= r_hat*phi_hat
            weights.append(p)

    # Normalize the particle weights
    total_weight = sum(weights)
    normalized_weights = []
    
    if(total_weight != 0):
      for i in weights:
         new_weight = i / total_weight
         normalized_weights.append(new_weight)
         
      if(len(weights) != 0):
         measured_particles = np.random.choice(particles, PARTICLE_COUNT-5, replace = True, p = normalized_weights)
    
    else:
      #return np.random.choice(particles, PARTICLE_COUNT-5, replace = True)
      return particle.create_random(PARTICLE_COUNT, grid)
    return measured_particles

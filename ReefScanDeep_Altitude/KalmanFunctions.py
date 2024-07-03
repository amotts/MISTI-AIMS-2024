import numpy as np
from math import radians, cos, sin, asin, sqrt
import filterpy.kalman as kf
from collections import namedtuple

gaussian = namedtuple('Gaussian', ['mean', 'var'])
gaussian.__repr__ = lambda s: f'𝒩(μ={s[0]:.3f}, 𝜎²={s[1]:.3f})'

def predict(pos, movement):
    """Kalman Prediction step using named Tuple"""
    return gaussian(pos.mean + movement.mean, pos.var + movement.var)

def gaussian_multiply(g1, g2):
    """Multiplication of gaussians using named Tuple"""
    mean = (g1.var * g2.mean + g2.var * g1.mean) / (g1.var + g2.var)
    variance = (g1.var * g2.var) / (g1.var + g2.var)
    return gaussian(mean, variance)

def update(prior, likelihood):
    """Kalman Update step using named Tuple"""
    posterior = gaussian_multiply(likelihood, prior)
    return posterior


def applyKalman(measurements, depth_var=2, sensor_var=10, delta_depth=0, discard_extreme = True, upper_extreme=2, lower_extreme=5, percent_extreme=0.25, extreme_lim = 2):
    """
    Applies a simple 1D Kalman Filter on the measurements inputted.
     
    Initial guess is the first measurement. Units assumed to be meters/frame as appropriate

    Discounts data points meeting critera for 'extreme' unless/until 'extreme limit' number in a row are recorded 

    Parameters:
    ------------
    measurements - The signal to be filtered\\
    depth_var - The variance assumed for the depth (Default = 2) \\
    sensor_var - The variance assumed for the depth sensor(s) (Default = 10) \\
    delta_depth - The initial depth rate of change in m per frame (Default = 0) \\
    upper_extreme - The bounds for downwards single frame change (Default = 5) \\
    lower_extreme - The bounds for upwards single frame change (Default = 2) \\
    percent_extreme - The percentage of current depth above is considered extreme \\
    extreme_lim - The Number of extreme points ignored before assuming to be valid data \\
    """
    zs = measurements
    x0 = zs[0]
    x = gaussian(x0, x0**2)  # initial depth

    process_model = gaussian(delta_depth, depth_var)
    xs, predictions = [], []
    # perform Kalman filter on measurement z
    z_prior = zs[0]
    extreme_count = 0
    for z in zs:
        extreme = False
        if discard_extreme:
            if ((z > percent_extreme*z_prior) or (z>z_prior+upper_extreme) or (z<z_prior-lower_extreme) or (z>0)) and extreme_count < extreme_lim: # Discount data point if point is too much higher or lower than previous or if less than 25% depth of previous point CAN USE ALTITUDE AS CUTOFF FOR VALID DATA
                z = z_prior  
                extreme_count +=1 
                extreme = True 
                #print(extreme_count)
        prior = predict(x, process_model)
        likelihood = gaussian(z, sensor_var)
        x = update(prior, likelihood)
        # save results
        predictions.append(prior.mean)
        xs.append(x.mean)
        z_prior = z
        if not extreme:
            extreme_count = 0
    return(xs)


class RealTimeKalman:
    """

    """
    def __init__(self, depth_var=2, sensor_var=10, delta_depth=0, discard_extreme = True, upper_extreme=2, lower_extreme=5, extreme_lim = 2):
        """

        """
        self.depth_var = depth_var
        self.sensor_var =sensor_var
        self.delta_depth = delta_depth
        self.upper_extreme = upper_extreme
        self.lower_extreme = lower_extreme
        self.extreme_lim = extreme_lim
        self.discard_extreme = discard_extreme
        self.extreme_count = 0

        self.x = None #most recent position
        self.process_model = gaussian(delta_depth, depth_var)
        self.prev_z = None #previous raw input
    
    def __str__(self):
       return("Realtime Kalman Filter")
        ##TODO create better representation##

    def set_initial_values(self, x0):
        self.x = gaussian(x0, x0**2)
        self.prev_z = x0

    def run_kalman(self, z):
    # perform Kalman filter on measurement z
        extreme = False
        if self.discard_extreme:
            if ((z>self.prev_z+self.upper_extreme) or (z<self.prev_z-self.lower_extreme) or (z>0)) and self.extreme_count < self.extreme_lim: # Discount data point if point is too much higher or lower than previous or if less than 25% depth of previous point CAN USE ALTITUDE AS CUTOFF FOR VALID DATA
                z = self.prev_z  
                self.extreme_count +=1 
                extreme = True 
                #print(self.extreme_count)


        prior = predict(self.x, self.process_model)
        self.x = update(prior, gaussian(z, self.sensor_var))

        self.prev_z = z
        if not extreme:
            self.extreme_count = 0

        return(self.x.mean)
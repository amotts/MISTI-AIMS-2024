import numpy as np
from math import radians, cos, sin, asin, sqrt
import matplotlib.pyplot as plt
from DepthAnalysisFunctions import *
import filterpy.kalman as kf
from collections import namedtuple


gaussian = namedtuple('Gaussian', ['mean', 'var'])
gaussian.__repr__ = lambda s: f'𝒩(μ={s[0]:.3f}, 𝜎²={s[1]:.3f})'

def predict(pos, movement):
    return gaussian(pos.mean + movement.mean, pos.var + movement.var)

def gaussian_multiply(g1, g2):
    mean = (g1.var * g2.mean + g2.var * g1.mean) / (g1.var + g2.var)
    variance = (g1.var * g2.var) / (g1.var + g2.var)
    return gaussian(mean, variance)

def update(prior, likelihood):
    posterior = gaussian_multiply(likelihood, prior)
    return posterior

PhotoLog = PhotoLogData()
PhotoLog.initializeData('Deep_Photo_Logs', 165)
print(PhotoLog)
PhotoLog.checkDataQuality()
PhotoLog.applygaussianFilter(4)
PhotoLog.applyLowpass(0.4)

zs = PhotoLog.df['total depth'].values
x0 = zs[0]
x = gaussian(x0, x0**2)  # initial depth

depth_var = 2 #Variance in depth
sensor_var = 10 # Variance in sensor
delta_depth = 0 #depth 'Velocity' in m/frame

process_model = gaussian(delta_depth, depth_var)

xs, predictions = [], []
# perform Kalman filter on measurement z
z_prior = zs[0]
extreme_count = 0
for z in zs:
    extreme = False
    if ((z > 0.25*z_prior) or (z>z_prior+2) or (z<z_prior-5) or (z>0)) and extreme_count < 2: # Discount data point if point is too much higher or lower than previous or if less than 25% depth of previous point CAN USE ALTITUDE AS CUTOFF FOR VALID DATA
        z = z_prior  
        extreme_count +=1 
        extreme = True 
        print(extreme_count)
    prior = predict(x, process_model)
    likelihood = gaussian(z, sensor_var)
    x = update(prior, likelihood)
    # save results
    predictions.append(prior.mean)
    xs.append(x.mean)
    z_prior = z
    if not extreme:
        extreme_count = 0

plt.figure(2)
plt.plot(PhotoLog.df['cumulative_distance'], xs, label='Kalman Filtered')
PhotoLog.plotgaussianFilter_distance(2)
PhotoLog.plotTotal_distance(2, points=True)
# PhotoLog.plotLowPass(2)
plt.show()



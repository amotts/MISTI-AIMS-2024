from DepthAnalysisFunctions import *
from KalmanFunctions import *
import numpy as np
from scipy import interpolate as interp
from scipy.optimize import fsolve

xy_signal = namedtuple('Signal', ['x', 'y'])

def RealTimeFlightPath(ground, x_vel = 2, altitude = 2, max_y_vel = 0.45, Hz = 3, depth_var=2, sensor_var=10, delta_depth=0, discard_extreme = True, upper_extreme=2, lower_extreme=5, extreme_lim = 2):
    """
    Creates a simulated vehicle flight path from a filtered total ground signal and altitude.

    ground - a xy_signal tuple containing x and y information 

    x_vel = the velocity to travel in the x direction (Default = 2 m/s)

    altitude - target height above total bottom depth to fly (Default = 2 m/s)

    max_y_vel - the maximum absolute vertical speed the vehicle can travel (Default = 0.45 m/s)

    Hz - the sampling rate of the depth sensor (Default = 3 Hz)

    See input descriptions for Kalman filter
    """
    depth_sensor = interp.interp1d(ground.x, ground.y)
    dt = 1/Hz

    flight_x = np.arange(0, ground.x[-1], x_vel*dt)
    # flight_t = np.arange(0,ground.x[-1]/x_vel, dt)
    flight_y = np.zeros(len(flight_x))
    targ_flight_y = np.zeros(len(flight_x))
    flight_y[0] = ground.y[0] + altitude
    targ_flight_y[0] = ground.y[0] + altitude


    kf = RealTimeKalman(depth_var, sensor_var, delta_depth, discard_extreme, upper_extreme, lower_extreme, extreme_lim)
    kf.set_initial_values(flight_y[0])

    i=1
    while i < len(flight_y):
        depth_reading = depth_sensor(flight_x[i])
        new_y = kf.run_kalman(depth_reading)
        targ_flight_y[i] = new_y + altitude
        
        if abs(targ_flight_y[i]- flight_y[i-1])/dt > max_y_vel:
            flight_y[i] = flight_y[i-1] + dt * np.sign(targ_flight_y[i]-flight_y[i-1])*max_y_vel
        else:
            flight_y[i] = targ_flight_y[i]
        i += 1

    return(flight_x, flight_y)


def checkCollisions(ground, flight, plot_collisions=False, n=15):
    """
    CAUTION: Ensure the ground being used for collision checking is the intended ground 
    
    Checks and counts collisions given a ground signal and the flight path.

    plot_collisions - triggers the flight path, ground, and collisions to be plotted on figure n (Default False, n = 15)

    Returns total number of collisions occurances defined as how many unique occasions the flight path collides 
    """
    total_collisions = 0
    total_occasions = 0
    col_depths = []
    col_dist = []
    true_depth = interp.interp1d(ground.x, ground.y)

    prev_col = False
    for ind in range(len(flight.y)):
        ground_depth = true_depth(flight.x[ind])
        if flight.y[ind] <= ground_depth:
            col_depths.append(ground_depth)
            col_dist.append(flight.x[ind])
            total_collisions += 1
            if not prev_col:
                prev_col = True
        elif prev_col:
            total_occasions +=1
            prev_col = False
    print(f"Finished collision checking. A total of {total_collisions} were found along the fligh path on {total_occasions} occasions")
    
        
    if plot_collisions:
        plt.figure(n)
        plt.plot(flight.x, flight.y, label="Flight Path")
        plt.plot(ground.x, ground.y, label="Ground")
        plt.plot(col_dist, col_depths, 'r.' ,  markersize=10, label="Collisions")
        plt.xlabel('Distance Traveled')
        plt.ylabel('Depth (m)')
        plt.title('Depth vs Distance Traveled')
        plt.legend()
        plt.grid(True)


    return(total_occasions)


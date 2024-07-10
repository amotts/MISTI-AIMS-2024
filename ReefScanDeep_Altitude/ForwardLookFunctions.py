from DepthAnalysisFunctions import *
from KalmanFunctions import *
import numpy as np
from scipy import interpolate as interp
from scipy.optimize import fsolve

def ForwardLookPing(ground, current_pos, angle = 45):
    """
    Returns the x,y cordinates of the forward angled sonar ping from current position and sonar angle

    ground - ground signal - filtered produces better data but can accept raw

    current_pos - tuple of current x, y position of vehicle

    angle - angle of FLS from hoirzontal in a clockwise direction

    NB: Actual sonar would produce only a distance which the vehcile would need to use its position knowledge
    to determine a relative distance and depth of the object.
    """
    ground_interp = interp.interp1d(ground.x, ground.y)
    x0, y0 = current_pos
    angle_rad = np.radians(-angle)
    slope = np.tan(angle_rad)
    
    # Define the line function DOES NOT TAKE INTO ACCOUNT SONAR CONE
    def line_func(x):
        return y0 + slope * (x - x0)
    
    # Define the function to find the root of (i.e., intersection point)
    def find_intersection(x):
        return ground_interp(x) - line_func(x)
    
    # Initial guess for the intersection point (can be improved based on context)
    x_initial_guess = x0

    # Use fsolve to find the intersection point
    try: # Necessary to avoid errors where intersection value is outside range of data 
        x_intersect = fsolve(find_intersection, x_initial_guess)[0]
        y_intersect = ground_interp(x_intersect)
    except ValueError:
        return(float("NaN"), float("NaN"))
    return(x_intersect, y_intersect)


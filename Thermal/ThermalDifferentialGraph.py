import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp



def solve_thermal_ivp(T_initial):
    # Given constants
    rho_air = 1.225
    rho_glass = 4000
    rho_alum = 2700
    v_air = 0.000617
    v_glass = 0.000308
    v_alum = 0.003186 * 0.015
    c_air = 1005
    c_glass = 840
    c_alum = 900

    # Calculate alpha values
    alpha_internal = rho_air * v_air * c_air + 0.25*710
    alpha_glass = rho_glass * v_glass * c_glass
    alpha_alum = rho_alum * v_alum * c_alum

    # Heat transfer coefficients
    h_air = 10
    h_water = 50

    # Surface areas
    a_glass = 0.0162
    a_alum = 0.003186 

    # Define the system of ODEs
    def F(t, s):
        A = np.array([
            [-h_air / alpha_internal * (a_glass + a_alum), h_air * a_glass / alpha_internal, h_air * a_alum / alpha_internal, 0],
            [h_air * a_glass / alpha_glass, -a_glass / alpha_glass * (h_air + h_water), 0, h_water * a_glass / alpha_glass],
            [h_air * a_alum / alpha_alum, 0, -a_alum / alpha_alum * (h_air + h_water), h_water * a_alum / alpha_alum],
            [0, 0, 0, 0]
        ])
        b = np.array([4.5/alpha_internal, 0, 0, 0])
        return np.dot(A, s) + b


    # Time evaluation points
    t_eval = np.arange(0, 10000, 0.01)

    # Initial conditions
    initial_conditions = [T_initial,.8*(T_initial+30)/2,.8*(T_initial+30)/2, 30]

    # Solve the system of ODEs
    sol = solve_ivp(F, [0, 10000], initial_conditions, method="BDF",  t_eval=t_eval)
    return(sol.t, sol.y[0])

# # Plotting the results
# plt.figure(figsize=(10, 6))
# plt.plot(sol.t, sol.y[0], label='Temperature of Air')
# plt.plot(sol.t, sol.y[1], label='Temperature of Glass')
# plt.plot(sol.t, sol.y[2], label='Temperature of Aluminum')
# plt.plot(sol.t, sol.y[3], label='Temperature of Water')

# plt.xlabel('Time (s)')
# plt.ylabel('Temperature (°C)')
# plt.title('Temperature Change Over Time')
# plt.legend()
# plt.grid(True)
# plt.show()
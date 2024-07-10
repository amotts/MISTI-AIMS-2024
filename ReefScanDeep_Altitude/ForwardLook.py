from DepthAnalysisFunctions import*
from KalmanFunctions import *
from FlightModelingFunctions import *


log_number = 154
pl = PhotoLogData()
pl.initializeData('Deep_Photo_Logs', log_number)
pl.applygaussianFilter(discard_extreme=True)


ground_raw = xy_signal(pl.df['cumulative_distance'].values, pl.df['total depth'].values)
ground_smooth = xy_signal(pl.df['cumulative_distance'].values, pl.df['gauss filtered data'])
flight = xy_signal(*RealTimeFlightPath(ground_raw, 0.5, 3.5))


FL_x = []
FL_y = []
i=0
for ind in range(len(flight.y)):
    x,y = ForwardLookPing(ground_raw, (flight.x[ind], flight.y[ind]))
    FL_x.append(x)
    FL_y.append(y)

print(FL_x)

x_array = np.arange(0.1, 10, 0.2)
pl.plotTotal_distance(15, True)

plt.figure(15)
plt.plot(FL_x, FL_y, "r.")
# plt.plot(x0+x_array, line_func(x0+x_array), "r.")

plt.show()
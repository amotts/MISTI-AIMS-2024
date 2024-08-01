from DepthAnalysisFunctions import *

pl = PhotoLogData()
pl.initializeData('Deep_Photo_Logs', 171)
pl.applygaussianFilter(discard_extreme=True)
pl.applyKalmanFilter()
pl.createFlightPath('Kalman', 2, print_counter=True)

pl.plotTotal(2, True)
pl.plotgaussianFilter(2)
pl.plotKalman(2)
pl.plotFlight(2)

pl.checkCollisions(ground ="Gaussian", print_all = False, plot_time=True, n =15)

pl.createFlightPath('Kalman', 2, print_counter=True, max_vel = None)
pl.checkCollisions(ground ="Gaussian", print_all = False, plot_time=True, n =16)
plt.show()
from DepthAnalysisFunctions import *

PhotoLog = PhotoLogData()
print(PhotoLog)
PhotoLog.initializeData('Deep_Photo_Logs', 159)
print(PhotoLog)
PhotoLog.checkDataQuality()

PhotoLog.applygaussianFilter(3, discard_extreme=True)
PhotoLog.applyLowpass(0.4)
PhotoLog.applyKalmanFilter(sensor_var=2, depth_var=2)

# PhotoLog.plotLowPass_distance(14)
# PhotoLog.plotgaussianFilter_distance(14)
PhotoLog.plotKalman_distance(14)
PhotoLog.plotTotal_distance(14, points=True)
# PhotoLog.checkCollisions(ground ="Gaussian", path = "Kalman", altitude=2, plot_distance=True, n =15)

PhotoLog.applyKalmanFilter()
PhotoLog.plotKalman_distance(15)
PhotoLog.plotTotal_distance(15, points=True)

plt.show()


# logs = [2,57,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171]
# collisions = dict()
# for num in logs:
#     PhotoLog = PhotoLogData()
#     PhotoLog.initializeData('Deep_Photo_Logs', num)
#     print(PhotoLog)
#     PhotoLog.checkDataQuality()
#     PhotoLog.applygaussianFilter(sigma =3, discard_extreme=True)
#     PhotoLog.applyLowpass(0.4)
#     PhotoLog.applyKalmanFilter()
#     num_col = PhotoLog.checkCollisions(ground="Gaussian", path="Lowpass", altitude=5.5, print_all=False)
#     collisions[num] = num_col

# print(collisions)

# PhotoLog = PhotoLogData()
# PhotoLog.initializeData('Deep_Photo_Logs', 157)
# print(PhotoLog)
# PhotoLog.checkDataQuality()
# PhotoLog.applygaussianFilter(sigma =3, discard_extreme=True)
# PhotoLog.applyLowpass(0.4)
# PhotoLog.applyKalmanFilter()
# PhotoLog.checkCollisions(ground="Gaussian", path="Kalman", altitude=2, print_all=False, plot_distance=True, n=2)
# PhotoLog.plotKalman_distance(2)

# plt.show()
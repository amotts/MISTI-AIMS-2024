from DepthAnalysisFunctions import *

# pl = PhotoLogData()
# pl.initializeData('Deep_Photo_Logs', 171)
# print(pl)
# pl.checkDataQuality()

# veh_depth = gaussian_filter1d(-pl.df['pressure_depth'].values, 3)
# times = pl.df['Elapsed Time']
# vert_vel = []
# for i in range(len(veh_depth)-1):
#     vert_vel.append((veh_depth[i+1]-veh_depth[i])/(times[i+1]-times[i]))



# plt.figure()
# plt.plot(times[1:], vert_vel, "k.")
# plt.xlabel("Time")
# plt.ylabel("Vertical Velocity (m/s)")
# plt.grid()

# plt.figure(2)
# plt.plot(times,veh_depth, label="Gauss Filtered Depth")
# pl.plotPressure(2)

# plt.show()


logs = [57,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170]
max_vel = dict()

for num in logs:
    pl = PhotoLogData()
    pl.initializeData('Deep_Photo_Logs', num)
    print(pl)
    pl.checkDataQuality()

    veh_depth = gaussian_filter1d(-pl.df['pressure_depth'].values, 3)
    times = pl.df['Elapsed Time']
    vert_vel = []
    for i in range(len(veh_depth)-1):
        vert_vel.append((veh_depth[i+1]-veh_depth[i])/(times[i+1]-times[i]))

    max_vel[num]= (max(vert_vel),min(vert_vel))

print(max_vel)


plt.figure()
plt.hist(max_vel.values())
plt.show()
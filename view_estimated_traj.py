import numpy as np
import matplotlib.pyplot as plt


# Model pose estimation plot (not calibrated)
# It is useful to estiamte if the estimated model is right
f_estimated = open("trajectories/model_pose_uncalibrated.txt")
lines_estimated = f_estimated.read().splitlines()
x_model_estimated = []
y_model_estimated = []
for l in lines_estimated:
	tokens = l.split()
	x_model_estimated.append(float(tokens[0]))
	y_model_estimated.append(float(tokens[1]))

fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x_model_estimated, y_model_estimated)
ax.axis("equal")
plt.show()

import numpy as np
import matplotlib.pyplot as plt
import sys
f = open("dataset/dataset.txt")

lines = f.read().splitlines()

c = 0
x_tracker = []
y_tracker = []

x_model_pose = []
y_model_pose = []

for l in lines:
	c += 1
	if(c < 10):
		continue
	tokens = l.split(":")
	tracker_pose = tokens[-1].strip()
	xy_tracker = tracker_pose.split(" ")
	x_tracker.append(float(xy_tracker[0]))
	y_tracker.append(float(xy_tracker[1]))
	
	model_pose = tokens[-2].strip().split()
	x_model_pose.append(float(model_pose[0]))
	y_model_pose.append(float(model_pose[1]))		

x_np = np.asarray(x_tracker)
y_np = np.asarray(y_tracker)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x_tracker, y_tracker, marker=".", label='tracker pose')
ax.scatter(x_model_pose, y_model_pose, marker=".", label='model pose')
ax.legend()
ax.axis("equal")
plt.show()
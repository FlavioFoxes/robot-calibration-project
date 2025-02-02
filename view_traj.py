import numpy as np
import matplotlib.pyplot as plt
f = open("dataset/dataset.txt")
lines = f.read().splitlines()

c = 0
x = []
y = []
x_model = []
y_model = []
for l in lines:
	c += 1
	if(c < 10):
		continue
	tokens = l.split(":")
	tracker_pose = tokens[-1].strip()
	xy = tracker_pose.split(" ")
	x.append(float(xy[0]))
	y.append(float(xy[1]))

	model_pose = tokens[-2].strip()
	pos_model = model_pose.split()
	x_model.append(float(pos_model[0]))
	y_model.append(float(pos_model[1]))



ft = open("example.txt")
lines_t = ft.read().splitlines()
x_t = []
y_t = []
for l in lines_t:
	tokens = l.split()
	x_t.append(float(tokens[0]))
	y_t.append(float(tokens[1]))



x_np = np.asarray(x)
y_np = np.asarray(y)
x_model_np = np.asarray(x_model)
y_model_np = np.asarray(y_model)
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x_t, y_t)
ax.scatter(x_model, y_model)
ax.axis("equal")
plt.show()

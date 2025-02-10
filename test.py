import numpy as np
import matplotlib.pyplot as plt



def create_dataset():
    f = open("dataset/dataset.txt")
    lines = f.read().splitlines()

    c = 0
    ticks_steer = []
    ticks_traction = []
    timestamps = []

    for l in lines:
        c += 1
        if(c<10):
            continue
        tokens = l.split(":")

        string = tokens[2].strip().split()
        ticks_steer.append(int(string[0]))
        ticks_traction.append(int(string[1]))

    return ticks_steer, ticks_traction

def step(x, y, theta, tick_steer, tick_traction, next_tick_steer, next_tick_traction):
    # print(tick_steer)
    # print(next_tick_steer)
    # print(tick_traction)
    # print(next_tick_traction)

    
    
    k_steer = 0.1
    k_traction = 0.0106141
    baseline = 1.4
    steer_offset = 0
    max_steer = 8192
    max_traction = 5000

    steering_angle = np.pi / max_steer * tick_steer *k_steer
    s = 2*np.pi /max_traction * (next_tick_traction-tick_traction) *k_traction
    
    dtheta = s * np.sin(steering_angle) / baseline
    theta += dtheta


    dx = s * np.cos(steering_angle) * np.cos(theta)
    dy = s * np.cos(steering_angle) * np.sin(theta)

    x += dx
    y += dy

    return x, y, theta

def compute(x_init, y_init, theta_init, ticks_steer, ticks_traction):
    x_list = []
    y_list = []

    x = x_init
    y = y_init
    theta = theta_init
    x_list.append(x)
    y_list.append(y)
    for i in range(80, len(ticks_steer)-1):

        tick_steer = ticks_steer[i]
        tick_traction = ticks_traction[i]
        next_tick_steer = ticks_steer[i+1]
        next_tick_traction = ticks_traction[i+1]

        x, y, theta = step(x, y, theta, tick_steer, tick_traction, next_tick_steer, next_tick_traction)
        x_list.append(x)
        y_list.append(y)

    return x_list, y_list


if __name__=="__main__":
    ticks_steer, ticks_traction = create_dataset()
    x_list, y_list = compute(0, 0, 0, ticks_steer, ticks_traction)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(x_list, y_list)
    ax.axis("equal")
    plt.show()

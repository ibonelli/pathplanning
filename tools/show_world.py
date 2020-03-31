"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
	# https://docs.scipy.org/doc/numpy/user/basics.creation.html
import matplotlib.pyplot as plt
	# https://matplotlib.org/users/pyplot_tutorial.html
import subprocess

def main():
    fName = "world05"
    filePath = "./"

    # goal position [x, y]
    start = np.array([5, 5])
    goal = np.array([55, 55])

    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt(fName + ".csv")
    obs = 1.0

    plt.cla()
    plt.plot(start[0], start[1], "xg")
    plt.plot(goal[0], goal[1], "ob")
    # ob[:, 0] -> The full first row of the array (all X numbers)
    # ob[:, 1] -> The full second row of the array (all Y numbers)
    #plt.plot(ob[:, 0], ob[:, 1], "ok")
    for obx,oby in np.nditer([ob[:, 0], ob[:, 1]]):
        patch=plt.Circle((obx, oby), obs, color='black', fill=True)
        tmp=plt.gca()
        tmp.add_patch(patch)
    plt.axis("equal")
    plt.grid(True)
    # We save to a file
    plt.savefig(filePath + fName + ".png")

if __name__ == '__main__':
    main()

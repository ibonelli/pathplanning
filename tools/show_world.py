"""

Tool to show world in CSV descriptions

"""

import os
import sys
import numpy as np
	# https://docs.scipy.org/doc/numpy/user/basics.creation.html
import matplotlib.pyplot as plt
	# https://matplotlib.org/users/pyplot_tutorial.html

def main():
    # Cargando el archivo de descripcion del mundo a recorrer
    if(len(sys.argv) < 2):
        print('Ingresar el mundo a recorrer. Por ejemplo world1.csv')
        exit(-1)
    else:
        fname = sys.argv[1]

    # goal position [x, y]
    start = np.array([5, 5])
    goal = np.array([55, 55])

    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt(fname)
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
    base=os.path.basename(fname)
    plt.savefig(os.path.splitext(base)[0] + ".png")
    plt.show()

if __name__ == '__main__':
    main()

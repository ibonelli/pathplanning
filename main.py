"""

Potential Field based path planner

Ref:
    https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import logging

# Modules
import config
from navApfNavigation import ApfNavigation
from navLidar import LidarPoint, LidarLimits, PathWindow
from navTrapNavigation import TrapNavigation
from navMap import Map

show_animation = config.general['animation']
logging.basicConfig(filename=config.general['logFile'],level=config.general['logLevel'])

# START Class MapNewPath --------------------------------------------
class MapNewPath:
    def __init__(self, reso, rr, radius):
        self.reso = reso
        self.rr = rr
        self.vision_limit = radius
        self.angle = math.atan(rr / radius)
        self.angle_step = int(2 * math.pi / self.angle)
        self.motion = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]]
# END Class MapNewPath --------------------------------------------------

def main():
    # Cargando el archivo de descripcion del mundo a recorrer
    if(len(sys.argv) < 2):
        print('Ingresar el mundo a recorrer. Por ejemplo world1.csv')
        exit(-1)
    else:
        fname = sys.argv[1]

    print("potential_field_planning start")

    grid_size = 1  # potential grid size [m]
    robot_radius = 5.0  # robot radius [m]
    vision_limit = 15

    ob = np.loadtxt(fname)
    flx = ob[:, 0]  # file x position list [m]
    fly = ob[:, 1]  # file y position list [m]
    sx = flx[0]     # start x position [m]
    sy = fly[0]     # start y positon [m]
    gx = flx[1]     # goal x position [m]
    gy = fly[1]     # goal y position [m]
    ox = flx[2:]   # obstacle x position list [m]
    oy = fly[2:]   # obstacle y position list [m]

    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    # path generation
    myNavigation = ApfNavigation(grid_size, robot_radius)
    trap = TrapNavigation(grid_size, robot_radius, vision_limit)

    # Objects learned
    myMap = Map()
    myMap.set_params(grid_size, gx, gy, ox, oy)
    myMap.create()

    # Get Limits
    myLimits = LidarLimits(grid_size, vision_limit, 8)
    limits = myLimits.fetch_limits(sx, sy, ox, oy)
    myMap.set_map(myLimits.limit2map(myMap.get_map(), limits))

    stuck = False
    d, rx, ry, curdirx, curdiry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, True)
    rd, rx, ry = [d], [sx], [sy]

    limits = myLimits.fetch_limits(sx, sy, ox, oy)
    myMap.set_map(myLimits.limit2map(myMap.get_map(), limits))
    intrap = trap.detect(myMap, sx, sy, curdirx, curdiry)

    if show_animation:
        myNavigation.draw_heatmap(myNavigation.get_pmap())
        plt.plot(sx, sy, "*k")
        plt.plot(gx, gy, "*m")

    while d >= grid_size and not stuck:
        d, xp, yp, curdirx, curdiry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, False)
        rd.append(d)
        rx.append(xp)
        ry.append(yp)
        stuck = myNavigation.decide_status(rd)
        limits = myLimits.fetch_limits(xp, yp, ox, oy)
        myMap.set_map(myLimits.limit2map(myMap.get_map(), limits))
        intrap = trap.detect(myMap, xp, yp, curdirx, curdiry)

        #This blocks by intrap
        #if stuck or intrap:
        if stuck:
            motion_model = trap.propose_motion_model(myMap, xp, yp)
            if len(motion_model) < 1:
                print("We can no longer navigate")
                limits = myLimits.fetch_limits(xp, yp, ox, oy)
                print("Limits: " + str(limits))
                windows = myLimits.get_limit_windows(limits, xp, yp)
                print("Current windows:")
                for win in windows:
                    win.print()
                myMap.draw()
                limits = myLimits.lidar(xp, yp, ox, oy)
                myLimits.graph_limits(limits)
                exit(0)
            myNavigation.set_motion_model(motion_model)
            #org_gx = gx
            #org_gy = gy
            #gx = 
            #gy = 
            stuck = False

    myMap.save_map("map.json")
    print("Goal!!")

    if show_animation:
        # We save to a file
        base=os.path.basename(fname)
        plt.savefig(os.path.splitext(base)[0] + "_nav.png")
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")

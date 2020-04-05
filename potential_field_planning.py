"""

Potential Field based path planner

Ref:
    https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain

show_animation = True

def calc_potential_field(gx, gy, ox, oy, reso, rr):
    minx = min(ox)
    miny = min(oy)
    maxx = max(ox)
    maxy = max(oy)
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i in range(len(ox)):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)

    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")

    rx, ry, rd = [sx], [sy], [d]
    motion = get_motion_model()
    scount=0
    stuck = False
    while d >= reso and not stuck:
        minp = float("inf")
        minix, miniy = -1, -1
        for i in range(len(motion)):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]):
                p = float("inf")  # outside area
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rd.append(d)
        rx.append(xp)
        ry.append(yp)

        ## Checking if we get stuck...
        dif = rd[-2] - rd[-1]
        if dif < 0 and scount == 0:
            scount = 1
            rcheck = rd[-2]
            print("Stuck? : " + str(rcheck))
        elif scount != 0:
            if (rcheck - rd[-1]) <= reso and scount <= 3:
                scount += 1
                print("Still stuck... scount: " + str(scount))
            elif (rcheck - rd[-1]) <= reso and scount > 3:
                print("Now we are really stuck!!!")
                stuck = True
            elif (rcheck - rd[-1]) > reso:
                print("We got out of rcheck+reso area")
                scount = 0
            else:
                print("How did we get here? (scount!=0)")
        else:
            scount = 0

        if stuck:
            print("NEED TO TRY DIFFERENT DIRECTION HERE...")

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=200.0, cmap=plt.cm.Blues)


def main():
    # Cargando el archivo de descripcion del mundo a recorrer
    if(len(sys.argv) < 2):
        print('Ingresar el mundo a recorrer. Por ejemplo world1.csv')
        exit(-1)
    else:
        fname = sys.argv[1]

    print("potential_field_planning start")

    sx = 5.0  # start x position [m]
    sy = 5.0  # start y positon [m]
    gx = 55.0  # goal x position [m]
    gy = 55.0  # goal y position [m]
    grid_size = 1  # potential grid size [m]
    robot_radius = 5.0  # robot radius [m]

    ob = np.loadtxt(fname)
    ox = ob[:, 0]  # obstacle x position list [m]
    oy = ob[:, 1]  # obstacle y position list [m]

    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    # path generation
    rx, ry = potential_field_planning(
        sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

    if show_animation:
        # We save to a file
        base=os.path.basename(fname)
        plt.savefig(os.path.splitext(base)[0] + "_nav.png")
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")

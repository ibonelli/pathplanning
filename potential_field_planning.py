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

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain

show_animation = True

# START Class LidarPoint -----------------------------------------------
class LidarPoint:
    def __init__(self):
        self.angle = 0
        self.r = np.zeros(2)
        self.dist = 0
        self.col = False

    def print_values(self):
        print "angle: " + str(self.angle)
        print "oi: " + str(self.r)
        print "dist: " + str(self.dist)
        print "collision: " + str(self.col)
# END Class LidarPoint -------------------------------------------------

# START Class LidarLimits ----------------------------------------------
class LidarLimits:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.sensor_radius = grid_size * 10
        self.sensor_angle_steps = 8  # [rad]
        self.angle_step = 2 * math.pi / self.sensor_angle_steps
        self.debug_limit_fname = "limit.png"
        self.debug_graph_fname = "navigation.png"

    # We get the limit for each LIDAR point
    # We will have as many limits as self.sensor_angle_steps
    # We store x, y and d (size of the vector)
    def lidar(self, x, y, ox, oy):
        limit = []

        for i in xrange(self.sensor_angle_steps):
            p = LidarPoint()
            p.angle = self.angle_step * i
            rx = x + self.sensor_radius * math.cos(self.angle_step * i)
            ry = y + self.sensor_radius * math.sin(self.angle_step * i)
            r = np.array([rx, ry])
            p.col,p.r = self.lidar_limits(x, y, r, ox, oy)
            p.dist = math.sqrt(math.pow(p.r[0]-x,2)+math.pow(p.r[1]-y,2))
            limit.append(p)

        return limit

    # We get the limit for an specific angle.
    def lidar_limits(self, x, y, r, ox, oy):
        oi = []
        for obx,oby,obs in np.nditer([ox, oy, self.grid_size]):
            intersects, limit = self.detect_collision_point(np.array([x, y]), r, np.array([obx, oby]), obs)
            if (intersects):
                #print "Intersection at " + str(limit)
                oi.append(limit)
        if (len(oi) == 0):
            return False, r
        elif (len(oi) == 1):
            return True, oi[0]
        else:
            min_val = float("inf")
            val_to_return = None
            for p in oi:
                h = np.linalg.norm(p)
                if (h < min_val):
                    min_val = h
                    val_to_return = p
            return True, p

    def detect_collision_point(self, p1, p2, q, r):
        intersects, values = self.getSegmentCircleIntersection(p1,p2,q,r)
        if (intersects):
            if (len(values) == 1):
                return intersects, values[0]
            else:
                m1 = np.linalg.norm(values[0])
                m2 = np.linalg.norm(values[1])
                if(m1>m2):
                    return intersects, values[1]
                else:
                    return intersects, values[0]
        else:
            return intersects, None

    def getSegmentCircleIntersection(self, p1, p2, q, r):
        v = np.array([p2[0]-p1[0], p2[1]-p1[1]])
        a = v.dot(v)
        b = 2 * v.dot(np.array([p1[0]-q[0], p1[1]-q[1]]))
        c = p1.dot(p1) + q.dot(q) - 2 * p1.dot(q) - r**2
        disc = b**2 - 4 * a * c
        if disc < 0:
            return False, None
        sqrt_disc = math.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)

        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return False, None
        else:
            i = []
            if (0 <= t1 <= 1):
                i1 = v.dot(t1) + p1
                i.append(i1)
            if (0 <= t2 <= 1):
                i2 = v.dot(t2) + p1
                i.append(i2)
            return True, i

    # From the LIDAR list we get best possible paths
    def get_limits(self, limit):
        oi_list = []

        for l in limit:
            if (l.col == False):
                oi_list.append(l)

        return oi_list

    # Change motion model according to found limits and prior path
    def get_new_motion_model(self, limit):
        model = None
        return model

    # Graph LIDAR limit
    def graph_limits(self, limit):
        plt.savefig(self.debug_graph_fname)
        oi_list = self.get_limits(limit)
        plt.cla()
        for l in limit:
            plt.plot(l.r[0], l.r[1], "b.")
        if (len(oi_list) >= 1):
            for l in oi_list:
                plt.plot(l.r[0], l.r[1], "xr")
        plt.axis("equal")
        plt.grid(True)
        plt.savefig(self.debug_limit_fname)
# END Class LidarLimits ------------------------------------------------

# START Class ApfNavigation --------------------------------------------
class ApfNavigation:
    def __init__(self, reso, rr):
        self.reso = reso
        self.rr = rr
        self.motion = [[1, 0],
                      [0, 1],
                      [-1, 0],
                      [0, -1],
                      [-1, -1],
                      [-1, 1],
                      [1, -1],
                      [1, 1]]

    def calc_potential_field(self, gx, gy, ox, oy):
        minx = min(ox)
        miny = min(oy)
        maxx = max(ox)
        maxy = max(oy)
        xw = int(round((maxx - minx) / self.reso))
        yw = int(round((maxy - miny) / self.reso))

        # calc each potential
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            x = ix * self.reso + minx

            for iy in range(yw):
                y = iy * self.reso + miny
                ug = self.calc_attractive_potential(x, y, gx, gy)
                uo = self.calc_repulsive_potential(x, y, ox, oy)
                uf = ug + uo
                pmap[ix][iy] = uf

        return pmap, minx, miny

    def calc_attractive_potential(self, x, y, gx, gy):
        return 0.5 * KP * np.hypot(x - gx, y - gy)

    def calc_repulsive_potential(self, x, y, ox, oy):
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

        if dq <= self.rr:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * ETA * (1.0 / dq - 1.0 / self.rr) ** 2
        else:
            return 0.0

    def get_motion_model(self):
        return self.motion

    def set_motion_model(self, motion):
        self.motion = motion

    def reset_motion_model(self):
        self.motion = [[1, 0],
                      [0, 1],
                      [-1, 0],
                      [0, -1],
                      [-1, -1],
                      [-1, 1],
                      [1, -1],
                      [1, 1]]

    def decide_status(self, rd, stuck, scount, rcheck):
        ## Checking if we get stuck...
        dif = rd[-2] - rd[-1]
        if dif < 0 and scount == 0:
            scount = 1
            rcheck = rd[-2]
            print("Stuck? : " + str(rcheck))
        elif scount != 0:
            if (rcheck - rd[-1]) <= self.reso and scount <= 3:
                scount += 1
                print("Still stuck... scount: " + str(scount))
            elif (rcheck - rd[-1]) <= self.reso and scount > 3:
                print("Now we are really stuck!!!")
                stuck = True
            elif (rcheck - rd[-1]) > self.reso:
                print("We got out of rcheck+reso area")
                scount = 0
            else:
                print("How did we get here? (scount!=0)")
        else:
            scount = 0

        return stuck, scount, rcheck

    def potential_field_planning(self, sx, sy, gx, gy, ox, oy):
        # calc potential field
        pmap, minx, miny = self.calc_potential_field(gx, gy, ox, oy)

        # search path
        d = np.hypot(sx - gx, sy - gy)
        ix = round((sx - minx) / self.reso)
        iy = round((sy - miny) / self.reso)
        gix = round((gx - minx) / self.reso)
        giy = round((gy - miny) / self.reso)

        if show_animation:
            self.draw_heatmap(pmap)
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")

        rx, ry, rd = [sx], [sy], [d]
        motion = self.get_motion_model()
        scount= 0
        rcheck= None
        stuck = False
        while d >= self.reso and not stuck:
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
            xp = ix * self.reso + minx
            yp = iy * self.reso + miny
            d = np.hypot(gx - xp, gy - yp)
            rd.append(d)
            rx.append(xp)
            ry.append(yp)

            stuck, scount, rcheck = self.decide_status(rd, stuck, scount, rcheck)

            if stuck:
                myLimits = LidarLimits(self.reso)
                limits = myLimits.lidar(xp, yp, ox, oy)
                myLimits.graph_limits(limits)
                print("NEED TO TRY DIFFERENT DIRECTION HERE...")

            if show_animation:
                plt.plot(ix, iy, ".r")
                plt.pause(0.01)

        print("Goal!!")

        return rx, ry

    def draw_heatmap(self, data):
        data = np.array(data).T
        plt.pcolor(data, vmax=200.0, cmap=plt.cm.Blues)
# END Class ApfNavigation ----------------------------------------------

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
    myNavigation = ApfNavigation(grid_size, robot_radius)
    rx, ry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy)

    if show_animation:
        # We save to a file
        base=os.path.basename(fname)
        plt.savefig(os.path.splitext(base)[0] + "_nav.png")
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")

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
        self.robot_position_x = None
        self.robot_position_y = None

    def fetch_limits(self, x, y, ox, oy):
        #debug
        #print "x: " + str(x) + " | y: " + str(y)
        #print "ox: "
        #print ox
        #print "oy: "
        #print oy
        #raw_input("Limits! Press Enter to continue...")

        oi = []
        for obx,oby in np.nditer([ox, oy]):
            d = math.sqrt(math.pow(obx-x,2)+math.pow(oby-y,2))
            #print "Distance: " + str(d)
            if d < self.sensor_radius:
                oi.append([int(obx), int(oby)])

        #debug
        #print "oi: "
        #print oi

        return oi

        #debug
        #raw_input("Limits! Press Enter to continue...")

    # We get the limit for each LIDAR point
    # We will have as many limits as self.sensor_angle_steps
    # We store x, y and d (size of the vector)
    def lidar(self, x, y, ox, oy):
        limit = []
        self.robot_position_x = x
        self.robot_position_y = y

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
            if (l.col <> False):
                oi_list.append(l)

        return oi_list

    def get_freepath(self, limit):
        fp_list = []

        for l in limit:
            if (l.col == False):
                fp_list.append(l)

        return fp_list

    # Change motion model according to found limits and prior path
    def get_new_motion_model(self, x, y, limit):
        # Debug
        #print("limit: ")
        #print(limit)
        if (len(limit) >= 1):
            for l in limit:
                dx = int(l[0] - x)
                dy = int(l[1] - y)
                # Debug
                #print("x: " + str(dx) + " | y: " + str(dy))
        # TODO : Build model...
        model = None
        return model

    # Graph LIDAR limit
    def graph_limits(self, limit):
        plt.savefig(self.debug_graph_fname)
        oi_list = self.get_limits(limit)
        fp_list = self.get_freepath(limit)
        plt.cla()
        if (len(fp_list) >= 1):
            for l in fp_list:
                plt.plot(l.r[0], l.r[1], "xr")
        if (len(oi_list) >= 1):
            for l in oi_list:
                plt.plot(l.r[0], l.r[1], "b.")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Lidar")
        plt.savefig(self.debug_limit_fname)

    # Limits to map
    def limit2map(self, omap, limit):
        #limit = self.get_limits(path_limit)
        if (len(limit) >= 1):
            for l in limit:
                ox = int(l[0])
                oy = int(l[1])
                omap[ox][oy] = 1.0
        return omap
# END Class LidarLimits ------------------------------------------------

# START Class Map ------------------------------------------------
class Map:
    def __init__(self, reso, gx, gy, ox, oy):
        self.reso = reso
        self.minx = min(ox)
        self.miny = min(oy)
        self.maxx = max(ox)
        self.maxy = max(oy)
        if gx > self.maxx:
            self.maxx = gx
        if gy > self.maxy:
            self.maxy = gy
        self.xw = int(round((self.maxx - self.minx) / self.reso)) + 1
        self.yw = int(round((self.maxy - self.miny) / self.reso)) + 1
        self.map = None
        self.debug_map_fname = "map.png"

    def create(self):
        # calc each potential
        self.map = [[0.0 for i in range(self.yw)] for i in range(self.xw)]
        return self.map

    def get_map(self):
        return self.map

    def set_map(self, exmap):
        self.map = exmap

    def get_minx(self):
        return self.minx

    def set_minx(self, minx):
        self.minx = minx

    def get_miny(self):
        return self.miny

    def set_miny(self, miny):
        self.miny = miny

    def get_maxx(self):
        return self.maxx

    def set_maxx(self, maxx):
        self.maxx = maxx

    def get_maxy(self):
        return self.maxy

    def set_maxy(self, maxy):
        self.maxy = maxy

    def get_xw(self):
        return self.xw

    def set_xw(self, xw):
        self.xw = xw

    def get_yw(self):
        return self.yw

    def set_yw(self, yw):
        self.yw = yw

    def draw(self):
        plt.cla()
        for j in range(self.yw):
            for i in range(self.xw):
                if self.map[i][j] == 1.0:
                    plt.plot(i*self.reso, j*self.reso, "b.")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Map")

        plt.savefig(self.debug_map_fname)
# END Class Map --------------------------------------------------

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
        self.pmap = None
        self.minx = None
        self.miny = None
        self.ix = None
        self.iy = None
        self.gix = None
        self.giy = None
        self.scount = None
        self.rcheck = None
        self.stuck = False

    def calc_potential_field(self, gx, gy, ox, oy):
        myMap = Map(self.reso, gx, gy, ox, oy)
        pmap = myMap.create()

        for ix in range(myMap.get_xw()):
            x = ix * self.reso + myMap.get_minx()

            for iy in range(myMap.get_yw()):
                y = iy * self.reso + myMap.get_miny()
                ug = self.calc_attractive_potential(x, y, gx, gy)
                uo = self.calc_repulsive_potential(x, y, ox, oy)
                uf = ug + uo
                pmap[ix][iy] = uf

        myMap.set_map(pmap)
        return pmap, myMap.get_minx(), myMap.get_miny()

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

    def get_pmap(self):
        return self.pmap

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

    def decide_status(self,rd):
        ## Checking if we get stuck...
        dif = rd[-2] - rd[-1]
        if dif < 0 and self.scount == 0:
            self.scount = 1
            self.rcheck = rd[-2]
            print("Stuck? : " + str(self.rcheck))
        elif self.scount != 0:
            if (self.rcheck - rd[-1]) <= self.reso and self.scount <= 3:
                self.scount += 1
                print("Still stuck... scount: " + str(self.scount))
            elif (self.rcheck - rd[-1]) <= self.reso and self.scount > 3:
                print("Now we are really stuck!!!")
                self.stuck = True
            elif (self.rcheck - rd[-1]) > self.reso:
                print("We got out of rcheck+reso area")
                self.scount = 0
            else:
                print("How did we get here? (scount!=0)")
        else:
            self.scount = 0

        return self.stuck

    def potential_field_planning(self, sx, sy, gx, gy, ox, oy, start):
        if start:
            # calc potential field
            self.pmap, self.minx, self.miny = self.calc_potential_field(gx, gy, ox, oy)
            # search path
            d = np.hypot(sx - gx, sy - gy)
            # Search variables
            self.ix = round((sx - self.minx) / self.reso)
            self.iy = round((sy - self.miny) / self.reso)
            self.gix = round((gx - self.minx) / self.reso)
            self.giy = round((gy - self.miny) / self.reso)
            # Results list initialization
            self.rx, self.ry, self.rd = [sx], [sy], [d]
            # Local minimum detection vars
            self.scount = 0
            self.rcheck = None
            self.stuck = False

        motion = self.get_motion_model()

        minp = float("inf")
        self.minix, self.miniy = -1, -1
        for i in range(len(motion)):
            inx = int(self.ix + motion[i][0])
            iny = int(self.iy + motion[i][1])
            if inx >= len(self.pmap) or iny >= len(self.pmap[0]):
                p = float("inf")  # outside area
            else:
                p = self.pmap[inx][iny]
            if minp > p:
                minp = p
                self.minix = inx
                self.miniy = iny

        self.ix = self.minix
        self.iy = self.miniy
        xp = self.ix * self.reso + self.minx
        yp = self.iy * self.reso + self.miny
        d = np.hypot(gx - xp, gy - yp)

        if show_animation:
            plt.plot(self.ix, self.iy, ".r")
            plt.pause(0.01)

        return d, xp, yp

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

    # Objects learned
    myMap = Map(grid_size, gx, gy, ox, oy)
    myMap.create()

    # Get Limits
    myLimits = LidarLimits(grid_size)
    limits = myLimits.fetch_limits(sx, sy, ox, oy)
    myMap.set_map(myLimits.limit2map(myMap.get_map(), limits))

    stuck = False
    d, rx, ry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, True)
    rd, rx, ry = [d], [sx], [sy]
    limits = myLimits.fetch_limits(rx, ry, ox, oy)
    myMap.set_map(myLimits.limit2map(myMap.get_map(), limits))

    if show_animation:
        myNavigation.draw_heatmap(myNavigation.get_pmap())
        plt.plot(sx, sy, "*k")
        plt.plot(gx, gy, "*m")

    while d >= grid_size and not stuck:
        d, xp, yp = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, False)
        rd.append(d)
        rx.append(xp)
        ry.append(yp)
        stuck = myNavigation.decide_status(rd)
        limits = myLimits.fetch_limits(xp, yp, ox, oy)
        myMap.set_map(myLimits.limit2map(myMap.get_map(), limits))

        if stuck:
            myMap.draw()
            myLimits.get_new_motion_model(xp, yp, limits)
            print("NEED TO TRY DIFFERENT DIRECTION HERE...")

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

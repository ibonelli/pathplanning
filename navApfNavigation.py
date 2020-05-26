import numpy as np
import matplotlib.pyplot as plt
import logging

# Modules
from navMap import Map

class ApfNavigation:
    def __init__(self, reso, rr):
        # Define model constants
        self.KP = 5.0  # attractive potential gain
        self.ETA = 100.0  # repulsive potential gain
        self.motion = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]]
        # Define object properties
        self.reso = reso
        self.rr = rr
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
        self.curdirx = None
        self.curdiry = None

    def calc_potential_field(self, gx, gy, ox, oy):
        myMap = Map()
        myMap.set_params(self.reso, gx, gy, ox, oy)
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
        return 0.5 * self.KP * np.hypot(x - gx, y - gy)

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

            return 0.5 * self.ETA * (1.0 / dq - 1.0 / self.rr) ** 2
        else:
            return 0.0

    def get_pmap(self):
        return self.pmap

    def get_motion_model(self):
        return self.motion

    def get_cur_dir(self):
        return self.curdirx, self.curdiry

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
            logging.debug("Stuck? : " + str(self.rcheck))
        elif self.scount != 0:
            if (self.rcheck - rd[-1]) <= self.reso and self.scount <= 3:
                self.scount += 1
                logging.debug("Still stuck... scount: " + str(self.scount))
            elif (self.rcheck - rd[-1]) <= self.reso and self.scount > 3:
                logging.info("Now we are really stuck!!!")
                self.stuck = True
            elif (self.rcheck - rd[-1]) > self.reso:
                logging.debug("We got out of rcheck+reso area")
                self.scount = 0
            else:
                logging.error("How did we get here? (scount!=0)")
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
                self.curdirx = motion[i][0]
                self.curdiry = motion[i][1]

        self.ix = self.minix
        self.iy = self.miniy
        xp = self.ix * self.reso + self.minx
        yp = self.iy * self.reso + self.miny
        d = np.hypot(gx - xp, gy - yp)

        return d, xp, yp, self.curdirx, self.curdiry

    def draw_heatmap(self, data):
        data = np.array(data).T
        plt.pcolor(data, vmax=200.0, cmap=plt.cm.Blues)

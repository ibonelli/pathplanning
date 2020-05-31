import math
import numpy as np
import matplotlib.pyplot as plt
import logging

# START Class LidarPoint ----------------------------------------------
class LidarPoint:
    def __init__(self):
        self.angle = 0
        self.r = np.zeros(2)
        self.dist = 0
        self.col = False

    def print_values(self):
        logging.info("angle: " + str(self.angle))
        logging.info("oi: " + str(self.r))
        logging.info("dist: " + str(self.dist))
        logging.info("collision: " + str(self.col))

    def get_coords(self):
        return list(self.r)

    def get_coord_list(self, limits_list):
        oi = []
        for aux in limits_list:
            coords = []
            mylist = aux.get_coords()
            coords.append(int(mylist[0]))
            coords.append(int(mylist[1]))
            oi.append(coords)
        return oi

# START Class LidarLimits ----------------------------------------------
class LidarLimits:
    def __init__(self, grid_size, radius, steps):
        self.grid_size = grid_size
        self.sensor_radius = grid_size * radius
        self.sensor_angle_steps = steps  # [rad]
        self.angle_step = 2 * math.pi / self.sensor_angle_steps
        self.debug_limit_fname = "limit.png"
        self.debug_graph_fname = "navigation.png"
        self.robot_position_x = None
        self.robot_position_y = None

    # Fetch limits (by inspection)
    #       (known problem is that detects through walls)
    def fetch_limits_by_inspection(self, x, y, ox, oy):
        oi = []
        for obx,oby in np.nditer([ox, oy]):
            d = math.sqrt(math.pow(obx-x,2)+math.pow(oby-y,2))
            #logging.debug("fetch_limits_by_inspection() | Distance: " + str(d))
            if d < self.sensor_radius:
                oi.append([int(obx), int(oby)])
        return oi

    # Fetch limits
    def fetch_limits(self, x, y, ox, oy, mode):
        limits = self.lidar(x, y, ox, oy, mode)
        oi_list = self.get_limits(limits)
        p = LidarPoint()
        oi = p.get_coord_list(oi_list)
        return oi

    # We get the limit for each LIDAR point
    # We will have as many limits as self.sensor_angle_steps
    # We store x, y and d (size of the vector)
    def lidar(self, x, y, ox, oy, mode):
        limit = []
        self.robot_position_x = x
        self.robot_position_y = y

        for i in range(self.sensor_angle_steps):
            p = LidarPoint()
            p.angle = self.angle_step * i
            rx = x + self.sensor_radius * math.cos(self.angle_step * i)
            ry = y + self.sensor_radius * math.sin(self.angle_step * i)
            r = np.array([rx, ry])
            p.col,p.r = self.lidar_limits(x, y, r, ox, oy, mode)
            p.dist = math.sqrt(math.pow(p.r[0]-x,2)+math.pow(p.r[1]-y,2))
            limit.append(p)

        return limit

    # We get the limit for an specific angle
    # Where:
    #      x,y: Is the position where LIDAR is measuring
    #        r: Is the current limit LIDAR position being explored for object collision
    #       ob: Is the x,y position of each object along with its radius (obs)
    def lidar_limits(self, x, y, r, ox, oy, mode):
        oi = []
        for obx,oby,obs in np.nditer([ox, oy, self.grid_size]):
            if mode == "object":
                intersects, limit = self.detect_collision_object(np.array([x, y]), r, np.array([obx, oby]), obs)
            elif mode == "limit":
                intersects, limit = self.detect_collision_point(np.array([x, y]), r, np.array([obx, oby]), obs)
            else:
                logging.error("Not proper mode: " + str(mode))
                exit(-1)
            if (intersects):
                #logging.debug("lidar_limits() | Intersection at " + str(limit))
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

    # We get the limit for an specific angle and return collision point.
    #   Builds a map with limits instead of objects (wich looks bad and leads to errors)
    #   Really useful to find obstacles in the way
    # Where:
    #       p1: Is the first point of a segment
    #       p2: Is the second point of a sagment
    #        q: Is the center of a circle that could intersect with the segment
    #        r: Is the radius of the circle that could intersect the segment
    def detect_collision_point(self, p1, p2, q, r):
        intersects, values = self.getSegmentCircleIntersection(p1,p2,q,r)
        # This code returns the proper intersection point
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

    # We get the object closest to the LIDAR looking angle.
    #   (Useful to build a map, but leads to errors when finding obstacles in the way)
    # Where:
    #       p1: Is the first point of a segment
    #       p2: Is the second point of a sagment
    #        q: Is the center of a circle that could intersect with the segment
    #        r: Is the radius of the circle that could intersect the segment
    def detect_collision_object(self, p1, p2, q, r):
        intersects, values = self.getSegmentCircleIntersection(p1,p2,q,r)
        # This code returns the actual objects (which simulation wise is better)
        if (intersects):
            return intersects, q
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

    # From the LIDAR list we get obstacles windows
    def get_limit_windows(self, limit, cX, cY):
        windows = []
        cur_window = PathWindow()
        total = len(limit)

        if total != 1:
            for i in range(total):
                l = limit[i]
                if i == 0:
                    cur_window.start = math.atan2(l.r[1] - cY, l.r[0] - cX)
                    init_type = cur_window.blocked = l.col
                elif i == total-1:
                    if init_type == cur_window.blocked:
                        if len(windows) == 0:
                            cur_window.start = cur_window.end = 0
                            windows.append(cur_window)
                        else:
                            windows[0].start = cur_window.start
                    else:
                        cur_window.end = math.atan2(l.r[1] - cY, l.r[0] - cX)
                        windows.append(cur_window)
                else:
                    if cur_window.blocked != l.col:
                        ang = math.atan2(l.r[1] - cY, l.r[0] - cX)
                        cur_window.end = ang
                        windows.append(cur_window)
                        cur_window = PathWindow()
                        cur_window.start = ang
                        cur_window.blocked = l.col
        else:
            logging.error("Only 1 limit")
            exit(-1)

        return windows

    # From the LIDAR list we get obstacles
    def get_limits(self, limit):
        oi_list = []

        for l in limit:
            if (l.col == True):
                oi_list.append(l)

        return oi_list

    # From the LIDAR list we get best possible paths
    def get_freepath(self, limit):
        fp_list = []

        for l in limit:
            if (l.col == False):
                fp_list.append(l)

        return fp_list

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

# START Class PathWindow -----------------------------------------------
class PathWindow:
    def __init__(self):
        self.blocked = False
        self.start = 0
        self.end = 0

    # From the LIDAR list we get obstacles windows
    def print(self):
        logging.debug("Blocked: " + str(self.blocked) + " | Start: " + str(math.degrees(self.start)) + " | End: " + str(math.degrees(self.end)))

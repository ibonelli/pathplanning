import math
import logging

# Modules
from navLidar import LidarLimits

# START Class TrapNavigation --------------------------------------------
class TrapNavigation:
    def __init__(self, reso, rr, radius):
        self.reso = reso
        self.rr = rr
        self.vision_limit = radius
        self.angle = math.atan(rr / radius)
        self.angle_step = int(2 * math.pi / self.angle)
        self.motion = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]]
        self.path_blocked_count = 0
        self.path_blocked_limit = 5
        self.col_status = False

    # We detect a collision in the robot path
    def detect(self, myMap, posX, posY, curdirx, curdiry):
        #logging.debug("angle_step: " + str(self.angle_step))
        myLimits = LidarLimits(self.reso, self.vision_limit, self.angle_step)
        rx = posX + self.vision_limit * curdirx
        ry = posY + self.vision_limit * curdiry
        ox, oy = myMap.get_objects()
        col,r = myLimits.lidar_limits(posX, posY, (rx, ry), ox, oy, "limit")
        #logging.debug("col: " + str(col))
        #logging.debug("r: " + str(r))

        if col:
            logging.info("Found obstacle in robots way...")
            limits = myLimits.lidar(posX, posY, ox, oy, "limit")
            windows = myLimits.get_limit_windows(limits, posX, posY)
            logging.debug("Current windows:")
            for win in windows:
                win.print()
            #myLimits.graph_limits(limits)
            self.col_status = True
            self.path_blocked_count+=1
        else:
            self.col_status == False
            self.path_blocked_count = 0

        if self.path_blocked_count == self.path_blocked_limit:
            blocked = True
        else:
            blocked = False

        return blocked

    def reset_motion_model(self):
        motionmodel = self.motion

    def propose_motion_model(self, myMap, posX, posY):
        myLimits = LidarLimits(self.reso, self.vision_limit, self.angle_step)
        ox, oy = myMap.get_objects()
        limits = myLimits.lidar(posX, posY, ox, oy, "limit")
        windows = myLimits.get_limit_windows(limits, posX, posY)
        logging.debug("Current windows:")
        for win in windows:
            win.print()
        # We new get new motion model
        proposed_motion_model = self.windows_to_motionmodel(windows)
        logging.debug("Proposed Motion Model:")
        logging.debug(proposed_motion_model)
        logging.debug("----------------------")
        return proposed_motion_model

    def windows_to_motionmodel(self, windows):
        new_motionmodel = []
        motionmodel = self.motion
        logging.debug("Windows to motion model:")
        for direction in motionmodel:
            ang = self.pangles(math.atan2(direction[1],direction[0]))
            logging.debug("For angle:" + str(math.degrees(self.pangles(ang))))
            for w in windows:
                w.print()
                if w.blocked == False:
                    if ang >= self.pangles(w.start) and ang <= self.pangles(w.end):
                        new_motionmodel.append([direction[0],direction[1]])
        return new_motionmodel

    # For the logic to work we need to have positive angles
    # But atan2() returns negatives as well
    def pangles(self,ang):
        if ang >= 0:
            return ang
        else:
            return (2 * math.pi + ang)

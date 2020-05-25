import math

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

    def detect(self, myMap, posX, posY, curdirx, curdiry):
        myLimits = LidarLimits(self.reso, self.vision_limit, self.angle_step)
        rx = posX + self.vision_limit * curdirx
        ry = posY + self.vision_limit * curdiry
        ox, oy = myMap.get_objects()
        col,r = myLimits.lidar_limits(posX, posY, (rx, ry), ox, oy)

        if col:
            print("Found obstacle in robots way...")
            limits = myLimits.lidar(posX, posY, ox, oy)
            windows = myLimits.get_limit_windows(limits, posX, posY)
            print("Current windows:")
            for win in windows:
                win.print()

        return True

    def propose_motion_model(self, myMap, posX, posY):
        myLimits = LidarLimits(self.reso, self.vision_limit, self.angle_step)
        ox, oy = myMap.get_objects()
        limits = myLimits.lidar(posX, posY, ox, oy)
        #myLimits.graph_limits(limits)
        windows = myLimits.get_limit_windows(limits, posX, posY)
        # We new get new motion model
        proposed_motion_model = self.windows_to_motionmodel(windows)
        return proposed_motion_model

    def windows_to_motionmodel(self, windows):
        new_motionmodel = []
        motionmodel = self.motion
        for direction in motionmodel:
            ang = self.pangles(math.atan2(direction[1],direction[0]))
            for w in windows:
                if w.blocked == False:
                    if ang > self.pangles(w.start) and ang < self.pangles(w.end):
                        new_motionmodel.append([direction[0],direction[1]])
        return new_motionmodel

    # For the logic to work we need to have positive angles
    # But atan2() returns negatives as well
    def pangles(self,ang):
        if ang > 0:
            return ang
        else:
            return (2 * math.pi + ang)

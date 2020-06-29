import math
import numpy as np
import logging

# Modules
from navLidar import Lidar
from navLidarLimit import LidarLimit, PathWindow

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
		self.path_blocked_limit = 3
		self.path_blocked_dir = np.zeros(2)
		self.col_status = False

	# We detect a collision in the robot path
	def detect(self, myMap, posX, posY, curdirx, curdiry, gx, gy):
		#logging.debug("angle_step: " + str(self.angle_step))
		myLidar = Lidar(self.reso, self.vision_limit, self.angle_step)
		rx = posX + self.vision_limit * curdirx
		ry = posY + self.vision_limit * curdiry
		ox, oy = myMap.get_objects()
		col,r = myLidar.lidar_limits(posX, posY, (rx, ry), ox, oy, "limit")
		#logging.debug("col: " + str(col))
		#logging.debug("r: " + str(r))

		# We look for collisions
		if col:
			dist_to_obstacle = np.linalg.norm(r)
			g = np.zeros(2)
			g[0] = posX - gx
			g[1] = posY - gy
			dist_to_goal = np.linalg.norm(g)
			if dist_to_obstacle < dist_to_goal:
				# To be a block, distance to obstacle must be less than distance to goal
				logging.info("Found obstacle in robots way...")
				limits = myLidar.lidar(posX, posY, ox, oy, "limit")
				myLimits = LidarLimit()
				windows = myLimits.get_limit_windows(limits, posX, posY)
				#logging.debug("Current windows:")
				#for win in windows:
				#	win.print()
				#myLimits.graph_limits(limits)
				logging.debug("Collission status is True!")
				self.col_status = True
				self.path_blocked_count += 1
				self.path_blocked_dir[0] = curdirx
				self.path_blocked_dir[1] = curdiry
			wall_detected = self.check_wall()
		else:
			self.path_blocked_dir = np.zeros(2)
			self.col_status = False
			self.path_blocked_count = 0
			wall_detected = False

		# If we have a collisions, we don't react immediately
		if self.path_blocked_count == self.path_blocked_limit:
			blocked = True
			blocked_dir = curdirx, curdiry
		else:
			blocked = False
			blocked_dir = None

		return blocked, blocked_dir, wall_detected

	def reset_motion_model(self):
		return self.motion

	# We will check the robots map of the world and use a higher resolution
	# The idea behind this is to have a resolution as high as the map can allow
	def propose_motion_model(self, myMap, posX, posY):
		myLidar = Lidar(self.reso, self.vision_limit, self.angle_step)
		ox, oy = myMap.get_objects()
		limits = myLidar.lidar(posX, posY, ox, oy, "limit")
		logging.debug("Limits:")
		for L in limits:
			L.print()
		myLimits = LidarLimit()
		windows = myLimits.get_limit_windows(limits, posX, posY)
		logging.debug("Current windows:")
		for win in windows:
			win.print("debug")
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
			logging.debug("\tFor angle:" + str(math.degrees(ang)))
			for w in windows:
				if w.blocked == False:
					if self.pangles(w.end) > self.pangles(w.start):
						logging.debug("\t\tNormal logic... | ang: " + str(math.degrees(ang)) + " | w_start:"  + str(math.degrees(self.pangles(w.start))) + " | w_end: " + str(math.degrees(self.pangles(w.end))))
						if ang >= self.pangles(w.start) and ang <= self.pangles(w.end):
							new_motionmodel.append([direction[0],direction[1]])
							logging.debug("\t\t\tValid direction: " + str((direction[0],direction[1])))
						else:
							logging.debug("\t\t\tNot valid direction.")
					else:
						logging.debug("\t\tOutside window logic... | ang: " + str(math.degrees(ang)) + " | w_start:"  + str(math.degrees(self.pangles(w.start))) + " | w_end: " + str(math.degrees(self.pangles(w.end))))
						if ang <= self.pangles(w.start) or ang >= self.pangles(w.end):
							new_motionmodel.append([direction[0],direction[1]])
							logging.debug("\t\t\tValid direction: " + str((direction[0],direction[1])))
						else:
							logging.debug("\t\t\tNot valid direction.")
		return new_motionmodel

	# For the logic to work we need to have positive angles
	# But atan2() returns negatives as well
	def pangles(self,ang):
		if ang >= 0:
			return ang
		else:
			return (2 * math.pi + ang)

	def check_wall(self):
		# TODO - Need to infer wall if there is a collision in the way
		return False

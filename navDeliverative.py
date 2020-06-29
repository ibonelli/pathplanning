import config
import math
import numpy as np
import logging
import sys

from navMap import Map
from navTrapNavigation import TrapNavigation
from navLidar import Lidar

lidar_steps = config.general['lidar_steps']

# START Class DelivNavigation --------------------------------------------
class DeliverativeNavigation:
	def __init__(self, robot_radius, vision_limit, grid_size, gx, gy, myMap):
		self.rr = robot_radius
		self.vision_limit = vision_limit
		self.grid_size = grid_size
		self.stuck = False
		self.path_blocked = False
		self.path_blocked_dir = None
		self.org_gx = gx
		self.org_gy = gy
		self.map = myMap
		self.path = []
		self.dir = []
		self.trap_dir = None

	def set_map(self, Map):
		self.map.set_map(Map)

	def get_map(self):
		return self.map.get_map()

	def get_map_obj(self):
		return self.map

	def set_step(self, step, direction, nav):
		self.path.append(step)
		self.dir.append(direction)
		logging.debug("step: " + str(step) + " | direction: " + str(direction) + " | navigation: " + str(nav))

	def set_status(self, stuck, path_blocked, path_blocked_dir):
		self.stuck = stuck
		self.path_blocked = path_blocked
		self.path_blocked_dir = path_blocked_dir

	def choose_dir(self, xp, yp):
		dirx = diry = None
		limitx = limity = None
		motion_model_progression = self.motion_model_progression(xp, yp)
		logging.debug("Deliverative --> Model progression: " + str(motion_model_progression))
		best_distance = sys.float_info[0]
		for m in motion_model_progression:
			if best_distance > m[2]:
				dirx, diry = m[0], m[1]
				best_distance = m[2]
		if dirx != None and diry != None:
			limitx = xp + self.vision_limit * dirx
			limity = yp + self.vision_limit * diry
		logging.debug("Chosen direction:")
		logging.debug("dirx = " + str(dirx) + " | diry = " + str(diry) + " | limitx = " + str(limitx) + " | limity = " + str(limity))
		logging.debug("----------------------")
		self.trap_dir = self.dir[-1]
		return dirx, diry, limitx, limity

	def motion_model_progression(self, xp, yp):
		trap = TrapNavigation(self.grid_size, self.rr, self.vision_limit)
		motion_model = trap.reset_motion_model()
		motion_model = trap.propose_motion_model(self.map, xp, yp)
		motion_model_progression = []
		for m in motion_model:
			x2 = xp + m[0] * self.vision_limit
			y2 = yp + m[1] * self.vision_limit
			d = np.hypot(self.org_gx - x2, self.org_gy - y2)
			m.append(d)
			motion_model_progression.append(m)
		return motion_model_progression

	def checked_path_blocked_dir(self, posX, posY):
		# TODO --- This is not working as it should...
		#          NEED TO ADD DEBUGGING AND FIGURE IT OUT WHAT'S WRONG!!!
		myLidar = Lidar(self.grid_size, self.vision_limit, lidar_steps)
		ox, oy = self.map.get_objects()
		col,r = myLidar.lidar_limits(posX, posY, self.trap_dir, ox, oy, "object")
		if col:
			# Bisector angle of two vectors in 2D
			# First we get each angle from original directions
			theta_u = math.atan2(self.trap_dir[0], self.trap_dir[1])
			theta_v = math.atan2(self.dir[-1][0], self.dir[-1][1])
			# Then we create a new vector with the average angle
			middle_theta = (theta_u+theta_v)/2
			m_dir = (math.cos(middle_theta), math.sin(middle_theta))
			# We now check if there is a block in that direction as well
			col,r = myLidar.lidar_limits(posX, posY, m_dir, ox, oy, "object")
			if col:
				# We keep original direction and set a new limit
				dirx = self.dir[-1][0]
				diry = self.dir[-1][1]
				limitx = posX + self.vision_limit * dirx
				limity = posY + self.vision_limit * diry
			else:
				# We set a new goal and limit it
				dirx = m_dir[0]
				diry = m_dir[1]
				limitx = posX + self.vision_limit * dirx
				limity = posY + self.vision_limit * diry
			logging.debug("Chosen direction:")
			logging.debug("dirx = " + str(dirx) + " | diry = " + str(diry) + " | limitx = " + str(limitx) + " | limity = " + str(limity))
			logging.debug("----------------------")
		else:
			# We keep original direction and set a new limit
			dirx = self.dir[-1][0]
			diry = self.dir[-1][1]
			limitx = posX + self.vision_limit * dirx
			limity = posY + self.vision_limit * diry
		return dirx, diry, limitx, limity

	def new_goal():
		return 0

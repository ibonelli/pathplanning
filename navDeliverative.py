import numpy as np
import logging
import sys

from navMap import Map
from navTrapNavigation import TrapNavigation
from navMap import Map

# START Class DelivNavigation --------------------------------------------
class DeliverativeNavigation:
	def __init__(self, robot_radius, vision_limit, grid_size, gx, gy, myMap):
		self.rr = robot_radius
		self.vision_limit = vision_limit
		self.grid_size = grid_size
		self.stuck = False
		self.path_blocked = False
		self.org_gx = gx
		self.org_gy = gy
		self.map = myMap
		self.path = []
		self.dir = []

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

	def set_status(self, stuck, path_blocked):
		self.stuck = stuck
		self.path_blocked = path_blocked

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

	def new_goal():
		return 0

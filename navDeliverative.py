import config
import math
import numpy as np
import logging
import sys

from navMap import Map
from navTrapNavigation import TrapNavigation
from navLidar import Lidar
from navBrushfire import BrushfireNavigation

lidar_steps = config.general['lidar_steps']
wall_detection_threshold = config.general['wall_detection_threshold']

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
		self.minx = self.miny = None
		self.maxx = self.maxy = None
		self.path = []
		self.dir = []
		self.nav = []
		self.pot = []
		self.trap_dir = None
		self.last_apf_direction = None

	def set_map(self, Map):
		self.map.set_map(Map)
		self.minx = self.map.get_minx()
		self.miny = self.map.get_miny()
		self.maxx = self.map.get_maxx()
		self.maxy = self.map.get_maxy()

	def get_map(self):
		return self.map.get_map()

	def get_map_obj(self):
		return self.map

	def set_step(self, step, direction, nav, pot):
		stepx = int(round(step[0],0))
		stepy = int(round(step[1],0))
		self.path.append((stepx, stepy))
		self.dir.append(direction)
		self.nav.append(nav)
		self.nav.append(pot)
		logging.debug("step: " + str(step) + " | direction: " + str(direction) + " | navigation: " + str(nav))

	def set_status(self, stuck, path_blocked, path_blocked_dir):
		self.stuck = stuck
		self.path_blocked = path_blocked
		self.path_blocked_dir = path_blocked_dir

	def check_limits(self, xp, yp):
		abort = False
		if xp < self.minx or xp > self.maxx:
			abort = True
		if yp < self.miny or yp > self.maxy:
			abort = True
		if abort:
			print("We went outside of the navigation limits. Aborting...")

		return abort

	def choose_dir(self, xp, yp):
		dirx = diry = None
		limitx = limity = None
		motion_model_progression = self.motion_model_progression(xp, yp)
		logging.debug("Deliverative --> Model progression:")
		logging.debug("\t" + str(motion_model_progression))
		best_distance = sys.float_info[0]

		for m in motion_model_progression:
			if best_distance > m[2]:
				dirx, diry = m[0], m[1]
				best_distance = m[2]

		if dirx != None and diry != None:
			limitx = xp + self.vision_limit * dirx
			limity = yp + self.vision_limit * diry
			# We need to check map limits
			if self.maxx < limitx:
				limitx = self.maxx - 1
			if self.maxy < limity:
				limity = self.maxy - 1
			if self.minx > limitx:
				limitx = self.minx - 1
			if self.miny > limity:
				limity = self.miny - 1

		logging.debug("Chosen direction:")
		logging.debug("\tdirx = " + str(dirx) + " | diry = " + str(diry) + " | limitx = " + str(limitx) + " | limity = " + str(limity))
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
			logging.debug("\t----------------------")
			logging.debug("\tChosen direction:")
			logging.debug("\tdirx = " + str(dirx) + " | diry = " + str(diry) + " | limitx = " + str(limitx) + " | limity = " + str(limity))
			logging.debug("\t----------------------")
		else:
			# We keep original direction and set a new limit
			dirx = self.dir[-1][0]
			diry = self.dir[-1][1]
			limitx = posX + self.vision_limit * dirx
			limity = posY + self.vision_limit * diry
		return dirx, diry, limitx, limity

	def is_following_wall(self, bmap):
		# TODO --- Make this flexible and dependent on wall_detection_threshold
		#	   --- Now it is not flexible. Below is a starting point...
		#
		#		status = False
		#		if len(self.path) > wall_detection_threshold:
		#			pot_cur = pot_prev = None
		#			all_equal = True
		#			for i in range(wall_detection_threshold):
		#				idx = -1
		#				if i == 0:
		#					pot_prev = bmap[self.path[-i][0]][self.path[-i][1]]
		#				else:
		#					pot_cur = bmap[self.path[-i][0]][self.path[-i][1]]
		#				
		#			if pot[0] == pot[1] and pot[1] == pot3:

		status = False
		if len(self.path) > wall_detection_threshold:
			pot1 = bmap[self.path[-1][0]][self.path[-1][1]]
			pot2 = bmap[self.path[-2][0]][self.path[-2][1]]
			pot3 = bmap[self.path[-3][0]][self.path[-3][1]]
			if pot1 == pot2 and pot2 == pot3:
				logging.debug("===================================")
				logging.debug("We are following an equipotential line!")
				logging.debug("\tCurrent potential: " + str(pot1))
				logging.debug("===================================")
				status = True
				self.pot = pot1

		return status

	def is_path_blocked(self):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		ox, oy = self.map.get_objects()
		rx = xp + self.vision_limit * self.dir[-1][0]
		ry = yp + self.vision_limit * self.dir[-1][1]
		myLidar = Lidar(self.grid_size, self.vision_limit, lidar_steps)
		col,r = myLidar.lidar_limits(xp, yp, (rx, ry), ox, oy, "limit")

		return col

	def decide_status(self, bmap):
		# TODO - Not finished, only working for certain status
		chosen_dir = None
		following_wall = self.is_following_wall(bmap)
		path_blocked = self.is_path_blocked()
		if following_wall and path_blocked:
			xp = self.path[-1][0]
			yp = self.path[-1][1]
			pot = bmap[self.path[-1][0]][self.path[-1][1]]
			# In this situation APF fails
			self.last_apf_direction = self.dir[-1]
			myNavWave = BrushfireNavigation()
			all_pot = myNavWave.get_motion_potentials(bmap, xp, yp)
			possible_dir = []
			for p in all_pot:
				if pot == p[2]:
					nextx = xp + p[0]
					nexty = xp + p[1]
					# Untraveled direction
					if nextx != self.path[-2][0] or nexty == self.path[-2][1]:
						possible_dir.append(p)
			if len(possible_dir) == 1:
				chosen_dir = (possible_dir[0][0], possible_dir[0][1])
			elif len(possible_dir) > 1:
					logging.error("More than one... Need to choose!")

		return chosen_dir

	def new_goal():
		return 0

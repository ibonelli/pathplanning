import math
import numpy as np
import logging
import sys

# Modules
import config
from navMap import Map
from navTrapNavigation import TrapNavigation
from navLidar import Lidar
from navLidarLimit import LidarLimit
from navBrushfire import BrushfireNavigation
from navData import NavigationData

lidar_steps = config.general['lidar_steps']
wall_detection_threshold = config.general['wall_detection_threshold']
brushfire_radius_explore = config.general['brushfire_radius_explore']
brushfire_radius_to_evaluate = config.general['brushfire_radius_to_evaluate']
brushfire_neighbors_limit = config.general['brushfire_neighbors_limit']
known_limit = config.general['known_limit']
block_size = config.general['block_size']
vision_limit = config.general['vision_limit']

# START Class DelivNavigation --------------------------------------------
class DeliverativeNavigation:
	def __init__(self, robot_radius, vision_limit, grid_size, gx, gy, myMap):
		self.rr = robot_radius
		self.vision_limit = vision_limit
		self.grid_size = grid_size
		self.org_goal = (gx, gy)
		self.new_goal = None
		self.map = myMap
		self.minx = self.miny = None
		self.maxx = self.maxy = None
		self.path = []
		self.dir = []
		self.nav = []
		self.pot = None
		self.nav_data = []
		self.trap_dir = None
		self.following_wall = None
		self.last_apf_direction = None
		self.last_apf_dist_to_obstacle = None

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

	def get_new_limits(self, dirx, diry):
		navData = self.nav_data[-1]
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		navdataval = navData.get_value(dirx,diry)
		if navdataval['limit_pos'] is None:
			xl, yl = xp+vision_limit*dirx, yp+vision_limit*diry
		else:
			xl, yl = navdataval['limit_pos']
		return (xl, yl)

	def set_step(self, step, direction, nav, pot, pvec, limits, navWave, navData=None):
		if navData==None:
			navData=NavigationData()
		stepx = int(round(step[0],0))
		stepy = int(round(step[1],0))
		self.path.append((stepx, stepy))
		self.dir.append(direction)
		self.nav.append(nav)
		for apf_pot, apf_dirx, apf_diry in pvec:
			# Building APF information
			navdataval = navData.build_info("pmap", apf_pot, navData.get_value(apf_dirx, apf_diry))
			navData.set_value(apf_dirx, apf_diry, navdataval)
			# Building blocking information
			myLimits = LidarLimit.get_limit_by_direction(limits, apf_dirx, apf_diry, 3)
			if myLimits['col'] == True:
				limit_pos = (int(round(myLimits['rx'])), int(round(myLimits['ry'])))
			else:
				limit_pos = None
			navdataval = navData.build_info("blocked", myLimits['col'], navData.get_value(apf_dirx, apf_diry))
			navdataval = navData.build_info("limit_pos", limit_pos, navData.get_value(apf_dirx, apf_diry))
			navData.set_value(apf_dirx, apf_diry, navdataval)
		# Building brushfire information
		navData = navWave.known_areas(stepx, stepy, brushfire_radius_explore, brushfire_radius_to_evaluate, brushfire_neighbors_limit, navData)
		self.nav_data.append(navData)
		logging.debug("step: " + str(step) + " | direction: " + str(direction) + " | navigation: " + str(nav))
		navData.print()

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
			d = np.hypot(self.org_goal[0] - x2, self.org_goal[1] - y2)
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

		# This only applies when we are following the original goal
		if self.new_goal != None:
			status = False

		return status

	def is_path_blocked(self):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		ox, oy = self.map.get_objects()
		rx = xp + self.vision_limit * self.dir[-1][0]
		ry = yp + self.vision_limit * self.dir[-1][1]
		myLidar = Lidar(self.grid_size, self.vision_limit, lidar_steps)
		col,r = myLidar.lidar_limits(xp, yp, (rx, ry), ox, oy, "object")
		d = np.hypot(r[0] - xp, r[1] - yp)
		return col, d

	def is_goal_unreachable(self, dist_to_obstacle):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		d = np.hypot(self.org_goal[0] - xp, self.org_goal[1] - yp)
		if dist_to_obstacle > d:
			return False
		else:
			return True

	def detect_trap(self):
		navData = self.nav_data[-1]
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		cur_dir = self.dir[-1]
		trap = False
		#for x,y in navData.get_iterative():
		#	dirVals = navData.get_value(x,y)
		#	if dirVals['blocked'] and dirVals['known'] > known_limit and dirVals['block_size'] > block_size:
		#		print("\t(" + str(x)  + "," + str(y) + ") -> " + str(dirVals))
		#		trap = True
		angle = math.atan2(cur_dir[1], cur_dir[0])
		x1 = int(round(math.cos(angle+math.pi/4)))
		y1 = int(round(math.sin(angle+math.pi/4)))
		dirVals1 = navData.get_value(x1,y1)
		if dirVals1['blocked'] and dirVals1['known'] > known_limit and dirVals1['block_size'] > block_size:
			x2 = int(round(math.cos(angle-math.pi/4)))
			y2 = int(round(math.sin(angle-math.pi/4)))
			dirVals2 = navData.get_value(x2,y2)
			if dirVals2['blocked'] and dirVals2['known'] > known_limit and dirVals2['block_size'] > block_size:
				trap = True
				logging.debug("====> Detect Trap Start")
				logging.debug("\tstep: (" + str(xp) + "," + str(yp) + ") | cur_dir: " + str(cur_dir))
				logging.debug("\t\t1.- (" + str(x1)  + "," + str(y1) + ") -> " + str(dirVals1))
				logging.debug("\t\t2.- (" + str(x2)  + "," + str(y2) + ") -> " + str(dirVals2))
				logging.debug("====< Detect Trap End")
		return trap

	def get_best_possible_path(self):
		navData = self.nav_data[-1]
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		cur_dir = self.dir[-1]
		new_dir = None
		possible_path = []
		gx, gy = self.org_goal
		logging.debug("====> Possible paths Start")
		for x,y in navData.get_iterative():
			dirVals = navData.get_value(x,y)
			if dirVals['blocked'] == False:
				possible_path.append((x,y,dirVals['pmap'],dirVals['known']))
		possible_path = sorted(possible_path, key=lambda mypath: mypath[2])
		for path in possible_path:
			logging.debug("\t(" + str(path[0]) + "," + str(path[1]) + ") | pot: " + str(path[2]) + " | known: " + str(path[3]))
		logging.debug("====< Possible paths End")
		return (possible_path[0][0], possible_path[0][1])

	def decide_status(self, bMap):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		# We will use all gathered information to decide what to do next
		chosen_dir = None
		goal_unreachable = False
		bmap = bMap.get_map()
		self.following_wall = self.is_following_wall(bmap)
		path_blocked, dist_to_obstacle = self.is_path_blocked()
		# If goal is within reach, there is no need to change direction
		if path_blocked:
			goal_unreachable = self.is_goal_unreachable(dist_to_obstacle)
			trap_detected = self.detect_trap()
		# If we have problems ahead, we need to decide what to do
		if path_blocked and goal_unreachable and (trap_detected or self.following_wall):
			pot = bmap[self.path[-1][0]][self.path[-1][1]]
			# In this situation APF fails
			self.last_apf_direction = self.dir[-1]
			self.last_apf_dist_to_obstacle = dist_to_obstacle
			chosen_dir = self.get_best_possible_path()
		return chosen_dir

	# Can set new goal if there is a block
	def unblock_status(self, nav):
		new_nav = nav
		newgx = self.org_goal[0]
		newgy = self.org_goal[1]
		curdirx = self.dir[-1][0]
		curdiry = self.dir[-1][1]
		if nav != "apf":
			if self.new_goal != None:
				logging.debug("unblock_status() : Reached new goal, moving back to APF.")
				xp = self.path[-1][0]
				yp = self.path[-1][1]
				if xp == self.new_goal[0] and yp == self.new_goal[1]:
					self.new_goal = None
					new_nav = "apf"
			else:
				xp = self.path[-1][0]
				yp = self.path[-1][1]
				ox, oy = self.map.get_objects()

				last_apf_ang = math.atan2(self.last_apf_direction[1], self.last_apf_direction[0])
				rx = xp + self.vision_limit * self.last_apf_direction[0]
				ry = yp + self.vision_limit * self.last_apf_direction[1]
				myLidar = Lidar(self.grid_size, self.vision_limit, lidar_steps)
				col1,r1 = myLidar.lidar_limits(xp, yp, (rx, ry), ox, oy, "limit")
				d1 = np.hypot(r1[0] - xp, r1[1] - yp)

				if d1 != self.last_apf_dist_to_obstacle:
					logging.debug("We are not following wall, distance has changed...")

				current_direction_ang = math.atan2(curdiry, curdirx)
				second_ang_to_check = (last_apf_ang + current_direction_ang) / 2
				logging.debug("unblock_status() => second_ang_to_check: " + str(math.degrees(second_ang_to_check)))
				rx = xp + self.vision_limit * math.cos(second_ang_to_check)
				ry = yp + self.vision_limit * math.sin(second_ang_to_check)
				myLidar = Lidar(self.grid_size, self.vision_limit, lidar_steps)
				col2,r2 = myLidar.lidar_limits(xp, yp, (rx, ry), ox, oy, "limit")
				d2 = np.hypot(r2[0] - xp, r2[1] - yp)

				if col2:
					new_nav = "follow"
				else:
					# We build direction (as 1s & 0s instead of angles)
					# This can be used as an angle2direction() method.
					xi = int(round(math.cos(second_ang_to_check)))
					yi = int(round(math.sin(second_ang_to_check)))
					# We calculate new goal
					new_nav = "follow"
					logging.debug("unblock_status() | xi: " + str(xi) + " | yi: " + str(yi))
					logging.debug("unblock_status() | last_apf_dist_to_obstacle: " + str(self.last_apf_dist_to_obstacle))
					newgx = xp + self.last_apf_dist_to_obstacle * xi
					newgy = yp + self.last_apf_dist_to_obstacle * yi
					curdirx = xi
					curdiry = yi
					logging.debug("unblock_status() : We have a new goal: (" + str(newgx) + "," + str(newgy) + ")")
					self.new_goal = (newgx, newgy)

		return curdirx, curdiry, newgx, newgy, new_nav

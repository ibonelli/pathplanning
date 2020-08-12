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
		self.pot = None
		self.nav = []
		self.nav_type = []
		self.nav_data = []
		self.trap_dir = None
		self.following_wall = None
		self.last_reactive_direction = None
		self.last_reactive_dist_to_obstacle = None

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

	def set_step(self, step, direction, nav, nav_type, pot, pvec, limits, navWave, navData=None):
		if navData==None:
			navData=NavigationData()
		stepx = int(round(step[0],0))
		stepy = int(round(step[1],0))
		self.path.append((stepx, stepy))
		self.dir.append(direction)
		self.nav.append(nav)
		self.nav_type.append(nav_type)
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
		logging.debug("step: " + str(step) + " | direction: " + str(direction) + " | navigation: " + str(nav) + " | mode: " + str(nav_type))
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
		trap = 0

		# -- Does not seem to require all directions
		#
		#for x,y in navData.get_iterative():
		#	dirVals = navData.get_value(x,y)
		#	if dirVals['blocked'] and dirVals['known'] > known_limit and dirVals['block_size'] > block_size:
		#		print("\t(" + str(x)  + "," + str(y) + ") -> " + str(dirVals))
		#		trap = True
		# --

		st_p45 = self.direction_status(cur_dir, math.pi/4, navData)
		st_n45 = self.direction_status(cur_dir, -math.pi/4, navData)
		# If we are blocked at front and 45 degree deviations
		if st_p45 == True and st_n45 == True:
			trap = 1
			# If we are blocked at front, 45 degree deviations and 90 degree
			st_p90 = self.direction_status(cur_dir, math.pi/2, navData)
			st_n90 = self.direction_status(cur_dir, -math.pi/2, navData)
			if st_p90 == True and st_n90 == True:
				trap = 2
		# Reporting...
		if trap != 0:
			logging.debug("====> Trap Detected")
			logging.debug("\tstep: (" + str(xp) + "," + str(yp) + ") | cur_dir: " + str(cur_dir) + " | type: " + str(trap))
			logging.debug("====< Detect Trap End")

		return trap

	def direction_status(self, cur_dir, ang, navData):
		cur_ang = math.atan2(cur_dir[1], cur_dir[0])
		x = int(round(math.cos(cur_ang+ang)))
		y = int(round(math.sin(cur_ang+ang)))
		dirVals = navData.get_value(x,y)
		status = False
		if dirVals['blocked'] and dirVals['known'] > known_limit and dirVals['block_size'] > block_size:
			status = True
		return status

	def get_best_possible_path(self):
		navData = self.nav_data[-1]
		possible_path = []
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
		newgx = self.org_goal[0]
		newgy = self.org_goal[1]
		curdirx = self.dir[-1][0]
		curdiry = self.dir[-1][1]
		# We will use all gathered information to decide what to do next
		decision_made = False
		goal_unreachable = False
		nav_changed = False
		cur_nav = self.nav[-1]
		nav_type = self.nav_type[-1]
		bmap = bMap.get_map()
		self.following_wall = self.is_following_wall(bmap)
		path_blocked, dist_to_obstacle = self.is_path_blocked()
		trap_detected = self.detect_trap()

		# If we find a trap type 2 -------------------------------------
		if path_blocked and trap_detected == 1:
			logging.debug("TRAP TYPE 1 DETECTED... Should be moving backwards now!")

		if path_blocked and trap_detected == 2:
			logging.debug("TRAP TYPE 2 DETECTED... Should be moving backwards now!")
			self.last_reactive_direction = self.dir[-1]
			self.last_reactive_dist_to_obstacle = dist_to_obstacle
			cur_nav = "follow"
			self.new_goal = self.get_new_limits(curdirx, curdiry)
			newgx, newgy = self.new_goal
			nav_changed, curdirx, curdiry, newgx, newgy, cur_nav = self.follow_wall_navigation()
			decision_made = True
			nav_changed = True
			nav_type = "deliverative"

		# If using apf navigation --------------------------------------
		if cur_nav == "apf":
			# If goal is within reach, there is no need to change direction
			if path_blocked:
				goal_unreachable = self.is_goal_unreachable(dist_to_obstacle)
				logging.debug("decide_status() for NAV: apf")
				logging.debug("\tpath_blocked: " + str(path_blocked) + " | goal_unreachable: " + str(goal_unreachable))
				logging.debug("\ttrap_detected: " + str(trap_detected) + " | following_wall: " + str(self.following_wall))

			# If we have problems ahead, we need to decide what to do
			if path_blocked and goal_unreachable and self.following_wall:
				# Only if we don't know enough from current direction
				if self.direction_status((curdirx, curdiry), 0, self.nav_data[-1]):
					pot = bmap[self.path[-1][0]][self.path[-1][1]]
					# In this situation APF fails
					self.last_reactive_direction = self.dir[-1]
					self.last_reactive_dist_to_obstacle = dist_to_obstacle
					curdirx, curdiry = self.get_best_possible_path()
					cur_nav = "follow"
					self.new_goal = self.get_new_limits(curdirx, curdiry)
					newgx, newgy = self.new_goal
					decision_made = True
					nav_changed = True
					nav_type = "deliverative"
				else:
					logging.debug("Path is blocked but we'll continue in this direction for a bit more...")
					logging.debug("\tNeed to know more about it. | navData: " + str(self.nav_data[-1].get_value(curdirx, curdiry)))

		# If using follow navigation -----------------------------------
		if decision_made == False and cur_nav == "follow":
			nav_changed, curdirx, curdiry, newgx, newgy, cur_nav, nav_type = self.follow_wall_navigation()

		return nav_changed, curdirx, curdiry, newgx, newgy, cur_nav, nav_type

	def follow_wall_navigation(self):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		newgx = self.org_goal[0]
		newgy = self.org_goal[1]
		curdirx = self.dir[-1][0]
		curdiry = self.dir[-1][1]
		nav_changed = False
		cur_nav = self.nav[-1]
		nav_type = self.nav_type[-1]

		if xp == self.new_goal[0] and yp == self.new_goal[1] and self.following_wall == False:
			logging.debug("decide_status() for NAV: follow")
			logging.debug("\tReached new goal, moving back to APF.")
			logging.debug("\tCurrent position: " + str((xp,yp)) + " | new_goal was: " + str(self.new_goal) + " | following_wall: " + str(self.following_wall))
			self.new_goal = None
			cur_nav = "apf"
			nav_changed = True
		else:
			ox, oy = self.map.get_objects()

			last_reactive_ang = math.atan2(self.last_reactive_direction[1], self.last_reactive_direction[0])
			rx = xp + self.vision_limit * self.last_reactive_direction[0]
			ry = yp + self.vision_limit * self.last_reactive_direction[1]
			myLidar = Lidar(self.grid_size, self.vision_limit, lidar_steps)
			col1,r1 = myLidar.lidar_limits(xp, yp, (rx, ry), ox, oy, "object")
			d1 = np.hypot(r1[0] - xp, r1[1] - yp)

			if d1 != self.last_reactive_dist_to_obstacle:
				logging.debug("decide_status() for NAV: follow")
				logging.debug("\tWe are not following wall, distance has changed...")
				logging.debug("\td1: " + str(d1) + " | last_reactive_dist: " + str(self.last_reactive_dist_to_obstacle))
				self.following_wall = False

			# We need to check if we reached the end of the obstacle we are following
			current_direction_ang = math.atan2(curdiry, curdirx)
			second_ang_to_check = (last_reactive_ang + current_direction_ang) / 2
			#logging.debug("decide_status() for NAV: follow")
			#logging.debug("\tsecond_ang_to_check: " + str(math.degrees(second_ang_to_check)))
			rx = xp + self.vision_limit * math.cos(second_ang_to_check)
			ry = yp + self.vision_limit * math.sin(second_ang_to_check)
			myLidar = Lidar(self.grid_size, self.vision_limit, lidar_steps)
			col2,r2 = myLidar.lidar_limits(xp, yp, (rx, ry), ox, oy, "object")
			d2 = np.hypot(r2[0] - xp, r2[1] - yp)

			if col2:
				cur_nav = "follow"
				if self.following_wall:
					self.new_goal = self.get_new_limits(curdirx, curdiry)
					newgx, newgy = self.new_goal
					logging.debug("decide_status() for NAV: follow")
					logging.debug("\tThe blocking object remains, extending the limit.")
					logging.debug("\tWe have a new goal: (" + str(newgx) + "," + str(newgy) + ")")
			else:
				# We reached the limit of the obstacle
				# We build direction (as 1s & 0s instead of angles)
				# This can be used as an angle2direction() method.
				xi = int(round(math.cos(second_ang_to_check)))
				yi = int(round(math.sin(second_ang_to_check)))
				# We calculate new goal
				cur_nav = "follow"
				newgx = xp + self.last_reactive_dist_to_obstacle * xi
				newgy = yp + self.last_reactive_dist_to_obstacle * yi
				curdirx = xi
				curdiry = yi
				logging.debug("decide_status() for NAV: follow")
				logging.debug("\txi: " + str(xi) + " | yi: " + str(yi))
				logging.debug("\tlast_reactive_dist_to_obstacle: " + str(self.last_reactive_dist_to_obstacle))
				logging.debug("\tWe have a new goal: (" + str(newgx) + "," + str(newgy) + ")")
				self.new_goal = (newgx, newgy)
				nav_changed = True
				nav_type = "deliverative"

		return nav_changed, curdirx, curdiry, newgx, newgy, cur_nav, nav_type

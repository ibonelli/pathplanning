import math
import numpy as np
import logging
import sys
import matplotlib.pyplot as plt

# Modules
import config
from navMap import Map
from navTrapNavigation import TrapNavigation
from navLidar import Lidar
from navLidarLimit import LidarLimit
from navBrushfire import BrushfireNavigation
from navData import NavigationData
from navAstar import AStarPlanner

lidar_steps = config.general['lidar_steps']
vision_limit = config.general['vision_limit']
wall_detection_threshold = config.general['wall_detection_threshold']
brushfire_radius_explore = config.general['brushfire_radius_explore']
brushfire_radius_to_evaluate = config.general['brushfire_radius_to_evaluate']
brushfire_neighbors_limit = config.general['brushfire_neighbors_limit']
known_limit = config.general['known_limit']
block_size = config.general['block_size']
trap_limit_distance = config.general['trap_limit_distance']
navData_debug = config.general['navData_debug']
show_Astar_animation = config.general['show_Astar_animation']

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
		self.pot = []
		self.nav = []
		self.nav_type = []
		self.nav_data = []
		self.trap_dir = None
		self.following_wall = None
		self.blocked_direction_to_overcome = None
		self.last_dist_to_obstacle = None
		self.wall_overcome = False
		self.before_trap_pos = None
		self.avoiding_trap = False
		self.avoiding_trap_type = 0
		self.finding_unknown = False

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
		self.pot.append(pot)
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
			# Checking known points in the chosen direction
			known_dir, max_potential, max_potential_point = navWave.known_point_direction(stepx, stepy, apf_dirx, apf_diry, 7)
			navdataval = navData.build_info("known_dir", known_dir, navData.get_value(apf_dirx, apf_diry))
			navdataval = navData.build_info("max_potential", max_potential, navData.get_value(apf_dirx, apf_diry))
			navdataval = navData.build_info("max_potential_point", max_potential_point, navData.get_value(apf_dirx, apf_diry))
			navData.set_value(apf_dirx, apf_diry, navdataval)
		# Building brushfire information
		navData = navWave.known_areas(stepx, stepy, brushfire_radius_explore, brushfire_radius_to_evaluate, brushfire_neighbors_limit, navData)
		# We save the data of the step
		self.nav_data.append(navData)
		logging.debug("step: " + str(step) + " | direction: " + str(direction) + " | navigation: " + str(nav) + " | mode: " + str(nav_type))
		if navData_debug:
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

	def is_following_wall(self):
		status = None

		if len(self.path) > wall_detection_threshold:
			if self.pot[-1] == self.pot[-2]:
				status = True
			else:
				status = False

			for i in range(wall_detection_threshold-2):
				if self.pot[-2-i] == self.pot[-3-i]:
					status = status and True
				else:
					status = status and False

			if status:
				logging.debug("===================================")
				logging.debug("We are following an equipotential line!")
				logging.debug("\tCurrent potential: " + str(self.pot[-1]))
				logging.debug("===================================")

		if status is None:
			status = False

		return status

	def is_path_blocked(self):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		navData = self.nav_data[-1]
		dirx = self.dir[-1][0]
		diry = self.dir[-1][1]
		dirVals = navData.get_value(dirx,diry)
		if dirVals['blocked'] == True:
			r = dirVals['limit_pos']
			d = np.hypot(r[0] - xp, r[1] - yp)
			b = dirVals['block_size']
		else:
			r = (float("inf"), float("inf"))
			d = float("inf")
			b = 0
		return dirVals['blocked'], d, b, r

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

		if trap !=0:
			logging.debug("\tTrap type " + str(trap) + " detected.")

		return trap

	def direction_status(self, cur_dir, ang, navData):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		cur_ang = math.atan2(cur_dir[1], cur_dir[0])
		x = int(round(math.cos(cur_ang+ang)))
		y = int(round(math.sin(cur_ang+ang)))
		dirVals = navData.get_value(x,y)
		status = False
		if dirVals['blocked'] and dirVals['known'] > known_limit and dirVals['block_size'] > block_size:
			d = np.hypot(xp - dirVals['limit_pos'][0], yp - dirVals['limit_pos'][1])
			if d < trap_limit_distance:
				status = True
		return status

	def explore_unknown(self):
		navData = self.nav_data[-1]
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		cur_dir = self.dir[-1]
		possible_path = []

		logging.debug("=========================================================")
		logging.debug("\tChecking least known areas...")

		for x,y in navData.get_iterative():
			dirVals = navData.get_value(x,y)
			if dirVals['known_dir'] < known_limit and dirVals['block_size'] < block_size:
				logging.debug("\t(" + str(x)  + "," + str(y) + ") -> " + str(dirVals))
				possible_path.append((x, y, dirVals['pmap'], dirVals['known']))

		# We will choose least know area with lower pmap
		if len(possible_path) > 0:
			possible_path = sorted(possible_path, key=lambda mypath: mypath[2])
			best_dir = (possible_path[0][0], possible_path[0][1])
			found = True
		else:
			best_dir = None
			found = False

		logging.debug("=========================================================")

		return found, best_dir

	def get_next_unknown_goal(self, bMap):
		# start and goal position
		sx, sy = self.path[-1][0], self.path[-1][1] # Current position
		#sx, sy = self.before_trap_pos # Last traped position
		gx, gy = self.org_goal
		grid_size = self.grid_size
		robot_radius = self.rr
		ox, oy = self.map.get_objects()

		if show_Astar_animation:
			plt.cla()
			plt.plot(ox, oy, ".k")
			plt.plot(sx, sy, "og")
			plt.plot(gx, gy, "xb")
			plt.grid(True)
			plt.axis("equal")

		a_star = AStarPlanner(ox, oy, grid_size, robot_radius, bMap)
		rx, ry, newgx, newgy, known = a_star.planning(sx, sy, gx, gy)

		logging.debug("Astar produced an APF New Goal = (" + str(newgx) + "," + str(newgy) + ") with a known value of: " + str(known))

		if show_Astar_animation:
			plt.plot(rx, ry, "-r")
			plt.pause(0.001)
			plt.show()

		return newgx, newgy

	def get_best_possible_path(self):
		navData = self.nav_data[-1]
		possible_path = []
		logging.debug("==== Possible paths")
		for x,y in navData.get_iterative():
			dirVals = navData.get_value(x,y)
			if dirVals['blocked'] == False:
				possible_path.append((x,y,dirVals['pmap'],dirVals['known']))
		possible_path = sorted(possible_path, key=lambda mypath: mypath[2])
		possible_path_count = len(possible_path)
		for path in possible_path:
			logging.debug("\t(" + str(path[0]) + "," + str(path[1]) + ") | pot: " + str(path[2]) + " | known: " + str(path[3]))
		logging.debug("\tA total of " + str(possible_path_count) + " paths were found.")
		return possible_path[0][0], possible_path[0][1], possible_path

	#def get_blocked_direction_to_overcome(self, bMap):
	#	xp = self.path[-1][0]
	#	yp = self.path[-1][1]
	#	navData = self.nav_data[-1]
	#	dirx = self.dir[-1][0]
	#	diry = self.dir[-1][1]
	#	dirVals = navData.get_value(dirx,diry)
	#
	#	neighbors = None
	#	if dirVals['blocked'] == True:
	#		if dirVals['block_size'] > 2:
	#			xl, yl = dirVals['limit_pos']
	#			neighbors = bMap.get_neighbors_list(xl, yl, 3)
	#
	#	if neighbors is not None:
	#		logging.debug("\t=======================================")
	#		logging.debug("\t\tNeighbors: " + str(neighbors))
	#		logging.debug("\t=======================================")
	#
	#	return neighbors

	def get_unknown_direction(self, bMap):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		known_level = bMap.known_point(xp, yp, 5)
		got_dir, curdir = self.explore_unknown()
		logging.debug("Known level: " + str(known_level) + " | got_dir: " + str(got_dir) + " | curdir: " + str(curdir))
		if (known_level < known_limit) and got_dir:
			logging.debug("Current exploring path has failed us!")
			nav_changed = True
			curdirx, curdiry = curdir
			cur_nav = "follow"
			nav_type = "deliverative"
			self.new_goal = self.get_new_limits(curdirx, curdiry)
			newgx, newgy = self.new_goal
			self.finding_unknown = True
			logging.debug("\tLeast known direction found... | direction: " + str((curdirx, curdiry)) + " | new goal: " + str((newgx, newgy)))
		else:
			logging.debug("All known! Need new strategy...")
			self.new_goal = self.get_next_unknown_goal(bMap)
			newgx, newgy = self.new_goal
			nav_changed = True
			curdirx, curdiry = None, None
			decision_made = True
			trap_detected = 0
			self.avoiding_trap = False
			cur_nav = "apf"
			nav_type = "deliverative"
		return nav_changed, curdirx, curdiry, newgx, newgy, cur_nav, nav_type

	def get_unknown_direction_stuck(self, bMap):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		nav_type = self.nav_type[-1]
		known_level = bMap.known_point(xp, yp, 5)
		got_dir, curdir = self.explore_unknown()
		logging.debug("Known level: " + str(known_level) + " | got_dir: " + str(got_dir) + " | curdir: " + str(curdir))
		if nav_type == "apf" and got_dir:
			logging.debug("Current exploring path has failed us!")
			nav_changed = True
			curdirx, curdiry = curdir
			cur_nav = "follow"
			nav_type = "deliverative"
			self.new_goal = self.get_new_limits(curdirx, curdiry)
			newgx, newgy = self.new_goal
			self.finding_unknown = True
			logging.debug("\tLeast known direction found... | direction: " + str((curdirx, curdiry)) + " | new goal: " + str((newgx, newgy)))
		else:
			logging.debug("All known! Need new strategy...")
			self.new_goal = self.get_next_unknown_goal(bMap)
			newgx, newgy = self.new_goal
			nav_changed = True
			curdirx, curdiry = None, None
			decision_made = True
			trap_detected = 0
			self.avoiding_trap = False
			cur_nav = "apf"
			nav_type = "deliverative"
		return nav_changed, curdirx, curdiry, newgx, newgy, cur_nav, nav_type

	def get_unblock_direction(self, bMap, btype):
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		navData = self.nav_data[-1]
		newdirx, newdiry, count = self.get_best_possible_path()

		if btype == 1:
			blocked_distance = 0
			new_ang = math.atan2(newdiry, newdirx)
			logging.debug("get_unblock_direction() | btype == 1 | newdir: " + str((newdirx,newdiry)) + "| newang: " + str(math.degrees(new_ang)))

			ang1 = math.pi/2
			x1 = int(round(math.cos(new_ang+ang1)))
			y1 = int(round(math.sin(new_ang+ang1)))
			dirVals1 = navData.get_value(x1,y1)
			logging.debug("\tEscape dir1 | direction: " + str((x1, y1)) + " | PMAP: " + str(dirVals1['pmap']) + " | Blocked: " + str(dirVals1['blocked']))

			ang2 = -math.pi/2
			x2 = int(round(math.cos(new_ang+ang2)))
			y2 = int(round(math.sin(new_ang+ang2)))
			dirVals2 = navData.get_value(x2,y2)
			logging.debug("\tEscape dir2 | direction: " + str((x2, y2)) + " | PMAP: " + str(dirVals2['pmap']) + " | Blocked: " + str(dirVals2['blocked']))

			if dirVals1['pmap'] < dirVals2['pmap']:
				bang = new_ang+ang1/2
				if dirVals1['blocked']:
					blocked_distance = np.hypot(xp - dirVals1['limit_pos'][0], yp - dirVals1['limit_pos'][1])
			else:
				bang = new_ang+ang2/2
				if dirVals2['blocked']:
					blocked_distance = np.hypot(xp - dirVals2['limit_pos'][0], yp - dirVals2['limit_pos'][1])

			x = int(round(math.cos(bang)))
			y = int(round(math.sin(bang)))
			logging.debug("\tEscape dir | direction: " + str((x, y)) + " | blocked Angle: " + str(math.degrees(bang)))

		elif btype == 2:
			logging.debug("get_unblock_direction() | btype == 2")
			cur_dir = self.dir[-1]
			cur_ang = math.atan2(cur_dir[1], cur_dir[0])

			x1 = int(round(math.cos(cur_ang+math.pi*3/4)))
			y1 = int(round(math.sin(cur_ang+math.pi*3/4)))
			dirVals1 = navData.get_value(x1,y1)
			logging.debug("\tEscape dir1 | direction: " + str((x1, y1)) + " | PMAP: " + str(dirVals1['pmap']))

			x2 = int(round(math.cos(cur_ang-math.pi*3/4)))
			y2 = int(round(math.sin(cur_ang-math.pi*3/4)))
			dirVals2 = navData.get_value(x2,y2)
			logging.debug("\tEscape dir2 | direction: " + str((x2, y2)) + " | PMAP: " + str(dirVals2['pmap']))

			if round(dirVals1['pmap'],2) == round(dirVals2['pmap'],2):
				xk1 = xp + x1 * self.vision_limit
				yk1 = yp + y1 * self.vision_limit
				known1 = bMap.known_point(xk1, yk1, 3)
				logging.debug("\t\tKnown | direction: " + str((x1, y1)) + " | Known: " + str(known1))
				xk2 = xp + x2 * self.vision_limit
				yk2 = yp + y2 * self.vision_limit
				known2 = bMap.known_point(xk2, yk2, 3)
				logging.debug("\t\tKnown | direction: " + str((x2, y2)) + " | Known: " + str(known2))
				if known1 < known2:
					x = x1
					y = y1
					if dirVals1['blocked']:
						blocked_distance = np.hypot(xp - dirVals1['limit_pos'][0], yp - dirVals1['limit_pos'][1])
				else:
					x = x2
					y = y2
					if dirVals2['blocked']:
						blocked_distance = np.hypot(xp - dirVals2['limit_pos'][0], yp - dirVals2['limit_pos'][1])
			elif dirVals1['pmap'] < dirVals2['pmap']:
				x = x1
				y = y1
				if dirVals1['blocked']:
					blocked_distance = np.hypot(xp - dirVals1['limit_pos'][0], yp - dirVals1['limit_pos'][1])
			else:
				x = x2
				y = y2
				if dirVals2['blocked']:
					blocked_distance = np.hypot(xp - dirVals2['limit_pos'][0], yp - dirVals2['limit_pos'][1])

		else:
			logging.error("Wrong type given: " + str(btype))

		dirVals = navData.get_value(x,y)
		logging.debug("\tEscape direction: " + str((x, y)) + " | dir navData: " + str(dirVals))
		if dirVals['blocked']:
			d = np.hypot(xp - dirVals['limit_pos'][0], yp - dirVals['limit_pos'][1])
		else:
			if blocked_distance != 0:
				d = (blocked_distance + 1) * math.sqrt(2)
			else:
				logging.info("\tThere is non-blocked direction, and no distance to overcome.")
				logging.info("\tSetting a distance 2 to simply avoid the situation.")
				d = 1

		return (x,y), (newdirx, newdiry), d

	def decide_status(self, bMap, stuck):
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
		self.following_wall = self.is_following_wall()
		path_blocked, dist_to_obstacle, limit_size, limit_pos = self.is_path_blocked()
		trap_detected = self.detect_trap()
		goal_unreachable = self.is_goal_unreachable(dist_to_obstacle)

		if nav_type == "deliverative" and cur_nav == "follow":
			if self.avoiding_trap:
				logging.debug("Avoding trap...")
				unblock, curdirx, curdiry, newgx, newgy = self.block_check()
				if unblock:
					decision_made == True 
					logging.debug("Moving back from a trap status! | direction: " + str((curdirx, curdiry)) + " | new goal: " + str((newgx, newgy)))
					nav_changed = True
					cur_nav = "follow"
					nav_type = "deliverative"
					self.avoiding_trap = False
					self.new_goal = (newgx, newgy)
				else:
					if path_blocked and dist_to_obstacle < trap_limit_distance:
						nav_changed, curdirx, curdiry, newgx, newgy, cur_nav, nav_type = self.get_unknown_direction(bMap)
					else:
						# If we are following and there is no block, we update the goal
						nav_changed = True
						self.new_goal = self.get_new_limits(curdirx, curdiry)
						newgx, newgy = self.new_goal
						cur_nav = "follow"
						nav_type = "deliverative"
			else:
				logging.debug("Checking if we reached new goal...")
				if xp == self.new_goal[0] and yp == self.new_goal[1]:
					nav_changed = True
					self.new_goal = None
					curdirx, curdiry = None, None
					newgx, newgy = self.org_goal
					decision_made = True
					cur_nav = "apf"
					nav_type = "reactive"

		# If we find a trap -------------------------------------
		if path_blocked and cur_nav == "apf" and (trap_detected == 1 or trap_detected == 2) and goal_unreachable:
			self.avoiding_trap = True
			self.avoiding_trap_type = trap_detected
			self.wall_overcome = False
			self.blocked_direction_to_overcome, newdir, self.last_dist_to_obstacle = self.get_unblock_direction(bMap, trap_detected)
			self.before_trap_pos = (xp, yp)
			curdirx, curdiry = newdir
			cur_nav = "follow"
			self.new_goal = self.get_new_limits(curdirx, curdiry)
			newgx, newgy = self.new_goal
			decision_made = True
			nav_changed = True
			nav_type = "deliverative"
			logging.debug("\tstep: " + str((xp,yp)) + ") | new_dir: " + str((newgx, newgy)) + " | trap_type: " + str(trap_detected) + " | blocked_direction_to_overcome: " + str(self.blocked_direction_to_overcome) + " | last_dist_to_obstacle: " + str(self.last_dist_to_obstacle))

		# Either APF or Follow has failed and returned stuck=True on decide_status() call
		if stuck:
			# APF failed!
			if cur_nav == "apf":
				logging.debug("Stuck between two Possible APF paths...")
			else:
				logging.debug("Follow got into a wall...")
			self.avoiding_trap = True
			self.avoiding_trap_type = 1
			self.wall_overcome = False
			self.blocked_direction_to_overcome, newdir, self.last_dist_to_obstacle = self.get_unblock_direction(bMap, self.avoiding_trap_type)
			self.before_trap_pos = (xp, yp)
			curdirx, curdiry = newdir
			cur_nav = "follow"
			self.new_goal = self.get_new_limits(curdirx, curdiry)
			newgx, newgy = self.new_goal
			decision_made = True
			nav_changed = True
			nav_type = "deliverative"
			logging.debug("\tstep: " + str((xp,yp)) + ") | new_dir: " + str((newgx, newgy)) + " | trap_type: " + str(trap_detected) + " | blocked_direction_to_overcome: " + str(self.blocked_direction_to_overcome) + " | last_dist_to_obstacle: " + str(self.last_dist_to_obstacle))

		return nav_changed, curdirx, curdiry, newgx, newgy, cur_nav, nav_type

	def block_check(self):
		ox, oy = self.map.get_objects()
		xp = self.path[-1][0]
		yp = self.path[-1][1]
		curdirx = self.dir[-1][0]
		curdiry = self.dir[-1][1]
		navData = self.nav_data[-1]

		# The angle we found is the one to check
		dirx = self.blocked_direction_to_overcome[0]
		diry = self.blocked_direction_to_overcome[1]
		dirVals = navData.get_value(dirx,diry)

		if dirVals['blocked'] == True:
			logging.debug("block_check() path unblock: False")
			unblock = False
			rx, ry = dirVals['limit_pos']
			newgx = self.org_goal[0]
			newgy = self.org_goal[1]
			newdirx = curdirx
			newdiry = curdiry
		else:
			logging.debug("block_check() path unblock: True")
			unblock = True
			self.wall_overcome = True
			newdirx = self.blocked_direction_to_overcome[0]
			newdiry = self.blocked_direction_to_overcome[1]
			ang = math.atan2(newdiry, newdirx)
			# Es la distancia al obstaculo, pero debemos superarlo para que APF vuelva a funcionar (por eso el segundo termino)
			newgx = int(round(xp + self.last_dist_to_obstacle * math.cos(ang))) + int(round(math.cos(ang)))
			newgy = int(round(yp + self.last_dist_to_obstacle * math.sin(ang))) + int(round(math.sin(ang)))

			logging.debug("We cleared the obstacle limit! | newdir: " + str((newdirx,newdiry)) + " | newgoal: " + str((newgx,newgy)))

		return unblock, newdirx, newdiry, newgx, newgy

	#def follow_wall_navigation(self):
	#	xp = self.path[-1][0]
	#	yp = self.path[-1][1]
	#	newgx = self.org_goal[0]
	#	newgy = self.org_goal[1]
	#	curdirx = self.dir[-1][0]
	#	curdiry = self.dir[-1][1]
	#	nav_changed = False
	#	cur_nav = self.nav[-1]
	#	nav_type = self.nav_type[-1]
	#
	#	if xp == self.new_goal[0] and yp == self.new_goal[1] and self.following_wall == False:
	#		logging.debug("decide_status() for NAV: follow")
	#		logging.debug("\tReached new goal, moving back to APF.")
	#		logging.debug("\tCurrent position: " + str((xp,yp)) + " | new_goal was: " + str(self.new_goal) + " | following_wall: " + str(self.following_wall))
	#		self.new_goal = None
	#		cur_nav = "apf"
	#		nav_type = "reactive"
	#		nav_changed = True
	#	else:
	#		ox, oy = self.map.get_objects()
	#
	#		last_reactive_ang = math.atan2(self.blocked_direction_to_overcome[1], self.blocked_direction_to_overcome[0])
	#		rx = xp + self.vision_limit * self.blocked_direction_to_overcome[0]
	#		ry = yp + self.vision_limit * self.blocked_direction_to_overcome[1]
	#		myLidar = Lidar(self.grid_size, self.vision_limit, lidar_steps)
	#		col1,r1 = myLidar.lidar_limits(xp, yp, (rx, ry), ox, oy, "object")
	#		d1 = np.hypot(r1[0] - xp, r1[1] - yp)
	#
	#		#if d1 != self.last_reactive_dist_to_obstacle:
	#		#	logging.debug("decide_status() for NAV: follow")
	#		#	logging.debug("\tWe are not following wall, distance has changed...")
	#		#	logging.debug("\td1: " + str(d1) + " | last_reactive_dist: " + str(self.last_reactive_dist_to_obstacle))
	#		#	self.following_wall = False
	#
	#		# We need to check if we reached the end of the obstacle we are following
	#		current_direction_ang = math.atan2(curdiry, curdirx)
	#		second_ang_to_check = (last_reactive_ang + current_direction_ang) / 2
	#		#logging.debug("decide_status() for NAV: follow")
	#		#logging.debug("\tsecond_ang_to_check: " + str(math.degrees(second_ang_to_check)))
	#		rx = xp + self.vision_limit * math.cos(second_ang_to_check)
	#		ry = yp + self.vision_limit * math.sin(second_ang_to_check)
	#		myLidar = Lidar(self.grid_size, self.vision_limit, lidar_steps)
	#		col2,r2 = myLidar.lidar_limits(xp, yp, (rx, ry), ox, oy, "object")
	#		d2 = np.hypot(r2[0] - xp, r2[1] - yp)
	#
	#		if col2:
	#			cur_nav = "follow"
	#			if self.following_wall:
	#				self.new_goal = self.get_new_limits(curdirx, curdiry)
	#				newgx, newgy = self.new_goal
	#				logging.debug("decide_status() for NAV: follow")
	#				logging.debug("\tThe blocking object remains, extending the limit.")
	#				logging.debug("\tWe have a new goal: (" + str(newgx) + "," + str(newgy) + ")")
	#		else:
	#			# We reached the limit of the obstacle
	#			# We build direction (as 1s & 0s instead of angles)
	#			# This can be used as an angle2direction() method.
	#			xi = int(round(math.cos(second_ang_to_check)))
	#			yi = int(round(math.sin(second_ang_to_check)))
	#			# We calculate new goal
	#			cur_nav = "follow"
	#			nav_type = "deliverative"
	#			self.following_wall = False
	#			newgx = xp + self.last_reactive_dist_to_obstacle * xi
	#			newgy = yp + self.last_reactive_dist_to_obstacle * yi
	#			self.wall_overcome = True
	#			curdirx = xi
	#			curdiry = yi
	#			logging.debug("decide_status() for NAV: follow")
	#			logging.debug("\txi: " + str(xi) + " | yi: " + str(yi))
	#			logging.debug("\tlast_reactive_dist_to_obstacle: " + str(self.last_reactive_dist_to_obstacle))
	#			logging.debug("\tWe have a new goal: (" + str(newgx) + "," + str(newgy) + ")")
	#			self.new_goal = (newgx, newgy)
	#			nav_changed = True
	#
	#	return nav_changed, curdirx, curdiry, newgx, newgy, cur_nav, nav_type

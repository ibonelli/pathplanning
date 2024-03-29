"""

A* grid planning

Ref:
	https://en.wikipedia.org/wiki/A*_search_algorithm
Original work by:
	https://github.com/AtsushiSakai/PythonRobotics

"""

import math
import matplotlib.pyplot as plt

import config
from navBrushfire import BrushfireNavigation

show_animation = config.general['show_Astar_animation']
known_limit = config.general['known_limit']
astar_known_explore_range = config.general['astar_known_explore_range']
astar_known_limit = config.general['astar_known_limit']

class AStarPlanner:
	def __init__(self, ox, oy, resolution, rr, bMap=None):
		"""
		Initialize grid map for a star planning

		ox: x position list of Obstacles [m]
		oy: y position list of Obstacles [m]
		resolution: grid resolution [m]
		rr: robot radius[m]
		"""

		self.resolution = resolution
		# TODO - This should be flexible. Some functions are, others are not.
		# Now working with 1 for Astar. Other sizes do not work!
		#self.rr = rr
		self.rr = 1
		self.obstacle_map = None
		self.motion = self.get_motion_model()

		if bMap is not None:
			# Importing information from bMap
			self.bMap = bMap
			limits = bMap.get_limits()
			self.min_x = limits['minx']
			self.min_y = limits['miny']
			self.max_x = limits['maxx']
			self.max_y = limits['maxy']
			# Making sure limits are withing obstacle points
			ox.append(limits['minx'])
			oy.append(limits['miny'])
			ox.append(limits['maxx'])
			oy.append(limits['maxy'])
		else:
			self.bMap = None
			self.min_x = int(round(min(ox),0))
			self.min_y = int(round(min(oy),0))
			self.max_x = int(round(max(ox),0))
			self.max_y = int(round(max(oy),0))

		self.x_width = round((self.max_x - self.min_x) / self.resolution)
		self.y_width = round((self.max_y - self.min_y) / self.resolution)
		self.calc_obstacle_map(ox, oy)

		# New goal to follow
		self.last_x = None
		self.last_y = None
		self.last_node = None

	class Node:
		def __init__(self, x, y, cost, parent_index):
			self.x = x  # index of grid
			self.y = y  # index of grid
			self.cost = cost
			self.parent_index = parent_index

		def __str__(self):
			return str(self.x) + "," + str(self.y) + "," + str(
				self.cost) + "," + str(self.parent_index)

	def planning(self, sx, sy, gx, gy):
		"""
		A star path search

		input:
			s_x: start x position [m]
			s_y: start y position [m]
			gx: goal x position [m]
			gy: goal y position [m]

		output:
			rx: x position list of the final path
			ry: y position list of the final path
		"""

		start_node = self.Node(self.calc_xy_index(sx, self.min_x),
							self.calc_xy_index(sy, self.min_y), 0.0, -1)

		goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
							self.calc_xy_index(gy, self.min_y), 0.0, -1)

		open_set, closed_set = dict(), dict()
		open_set[self.calc_grid_index(start_node)] = start_node

		while 1:
			if len(open_set) == 0:
				print("Open set is empty..")
				break

			c_id = min(
				open_set,
				key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
																	open_set[o]))
			current = open_set[c_id]

			# show graph
			if show_animation:  # pragma: no cover
				plt.plot(self.calc_grid_position(current.x, self.min_x),
						self.calc_grid_position(current.y, self.min_y), "xc")
				# for stopping simulation with the esc key.
				plt.gcf().canvas.mpl_connect('key_release_event',
											lambda event: [exit(
												0) if event.key == 'escape' else None])
				if len(closed_set.keys()) % 10 == 0:
					plt.pause(0.001)

			# Originally it ends when it finds the goal
			if current.x == goal_node.x and current.y == goal_node.y:
				print("Found goal")
				goal_node.parent_index = current.parent_index
				goal_node.cost = current.cost
				self.last_x = current.x
				self.last_y = current.y
				self.last_node = current
				known = 0
				break

			# In my usecase it is also useful to finish when it got to an unknown spot
			if self.bMap is not None:
				#print("current.x : " + str(current.x) + " | current.y : " + str(current.y))
				known = self.bMap.known_point(current.x, current.y, astar_known_explore_range)
				if known < astar_known_limit:
					goal_node.parent_index = current.parent_index
					goal_node.cost = current.cost
					self.last_x = current.x
					self.last_y = current.y
					self.last_node = current
					break

			# Remove the item from the open set
			del open_set[c_id]

			# Add it to the closed set
			closed_set[c_id] = current

			# expand_grid search grid based on motion model
			for i, _ in enumerate(self.motion):
				node = self.Node(current.x + self.motion[i][0],
								current.y + self.motion[i][1],
								current.cost + self.motion[i][2], c_id)
				n_id = self.calc_grid_index(node)

				# If the node is not safe, do nothing
				if not self.verify_node(node):
					continue

				if n_id in closed_set:
					continue

				if n_id not in open_set:
					open_set[n_id] = node  # discovered a new node
				else:
					if open_set[n_id].cost > node.cost:
						# This path is the best until now. record it
						open_set[n_id] = node

		if len(open_set) == 0:
			# Could not find a path
			return 0, 0, 0, 0, 0
		else:
			# Once finished, we calculate the path
			rx, ry = self.calc_final_path(self.last_node, closed_set)
			return rx, ry, self.last_x, self.last_y, known

	def calc_final_path(self, goal_node, closed_set):
		# generate final course
		rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
			self.calc_grid_position(goal_node.y, self.min_y)]
		parent_index = goal_node.parent_index
		while parent_index != -1:
			n = closed_set[parent_index]
			rx.append(self.calc_grid_position(n.x, self.min_x))
			ry.append(self.calc_grid_position(n.y, self.min_y))
			parent_index = n.parent_index

		return rx, ry

	@staticmethod
	def calc_heuristic(n1, n2):
		w = 1.0  # weight of heuristic
		d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
		return d

	def calc_grid_position(self, index, min_position):
		"""
		calc grid position
	
		:param index:
		:param min_position:
		:return:
		"""
		pos = index * self.resolution + min_position
		return pos

	def calc_xy_index(self, position, min_pos):
		return round((position - min_pos) / self.resolution)

	def calc_grid_index(self, node):
		return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

	def verify_node(self, node):
		px = self.calc_grid_position(node.x, self.min_x)
		py = self.calc_grid_position(node.y, self.min_y)

		if px < self.min_x:
			return False
		elif py < self.min_y:
			return False
		elif px >= self.max_x:
			return False
		elif py >= self.max_y:
			return False

		# collision check
		if self.obstacle_map[node.x][node.y]:
			return False

		return True

	def calc_obstacle_map(self, ox, oy):
		# obstacle map generation
		self.obstacle_map = [[False for _ in range(self.y_width)]
							for _ in range(self.x_width)]
		for ix in range(self.x_width):
			x = self.calc_grid_position(ix, self.min_x)
			for iy in range(self.y_width):
				y = self.calc_grid_position(iy, self.min_y)
				for iox, ioy in zip(ox, oy):
					d = math.hypot(iox - x, ioy - y)
					if d <= self.rr:
						self.obstacle_map[ix][iy] = True
						break

	@staticmethod
	def get_motion_model():
		# dx, dy, cost
		motion = [[1, 0, 1],
				[0, 1, 1],
				[-1, 0, 1],
				[0, -1, 1],
				[-1, -1, math.sqrt(2)],
				[-1, 1, math.sqrt(2)],
				[1, -1, math.sqrt(2)],
				[1, 1, math.sqrt(2)]]

		return motion

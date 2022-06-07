import math
import numpy as np
import matplotlib.pyplot as plt
import logging
import json
from navLidarPoint import LidarPoint
from navLidarLimit import LidarLimit

lidar_debug = False
lidar_intersection_debug = False
lidar_object_debug = False

# START Class Lidar ----------------------------------------------
class Lidar:
	def __init__(self, grid_size, radius, steps):
		self.grid_size = grid_size
		self.sensor_radius = grid_size * radius
		self.sensor_angle_steps = steps  # [rad]
		self.angle_step = 2 * math.pi / self.sensor_angle_steps
		self.debug_limit_fname = "limit.png"
		self.debug_graph_fname = "navigation.png"
		self.robot_position_x = None
		self.robot_position_y = None

	# Fetch limits (by inspection)
	#		(known problem is that detects through walls)
	def fetch_limits_by_inspection(self, x, y, ox, oy):
		oi = []
		for obx,oby in np.nditer([ox, oy]):
			d = math.sqrt(math.pow(obx-x,2)+math.pow(oby-y,2))
			#logging.debug("fetch_limits_by_inspection() | Distance: " + str(d))
			if d < self.sensor_radius:
				oi.append([int(obx), int(oby)])
		return oi

	# Fetch limits
	def fetch_all(self, x, y, ox, oy, mode):
		limits = self.lidar(x, y, ox, oy, mode)
		oi_list = LidarLimit.get_all(limits)
		p = LidarPoint()
		oi = p.get_coords_point_data(oi_list)
		if lidar_object_debug:
			logging.debug("fetch_all()-------------------------------")
			logging.debug("Object list => " + str(oi))
			logging.debug("---------------------------------------------")
		return oi

	def get_blocked_path(self, limits):
		object_list = []
		for l in limits:
			if (l["col"] == True):
				object_list.append([l["rx"],l["ry"]])
		return object_list

	# We get the limit for each LIDAR point
	# We will have as many limits as self.sensor_angle_steps
	# We store x, y and d (size of the vector)
	def lidar(self, x, y, ox, oy, mode):
		limit = []
		self.robot_position_x = x
		self.robot_position_y = y

		if lidar_debug:
			self.print("lidar() start ------------------", "debug")
		for i in range(self.sensor_angle_steps):
			p = LidarPoint()
			p.angle = self.angle_step * i
			rx = x + self.sensor_radius * math.cos(p.angle)
			ry = y + self.sensor_radius * math.sin(p.angle)
			r = np.array([rx, ry])
			if lidar_debug:
				logging.debug("-> Call " + str(i) + " | x: " + str(x) + " | y: " + str(y) + " | r " + str(r) + " | mode: " + mode)
			p.col,p.r = self.lidar_limits(x, y, r, ox, oy, mode)
			p.dist = math.sqrt(math.pow(p.r[0]-x,2)+math.pow(p.r[1]-y,2))
			if lidar_debug:
				self.print("---> col: " + str(p.col) + " | r: " + str(p.r) + " | ang: " + str(math.degrees(p.angle)) + " | x: " + str(p.r[0]-x) + " | y: " + str(p.r[1]-y) + " | d: " + str(p.dist), "debug")
			limit.append(p)

		if lidar_debug:
			self.print("lidar() limits:", "debug")
			for l in limit:
				self.print("----> col: " + str(l.col) + " | r: " + str(l.r) + " | dist: " + str(l.dist), "debug")
			self.print("lidar() end ------------------", "debug")
		return limit

	# We get the limit for an specific angle
	# Where:
	#	  x,y: Is the position where LIDAR is measuring
	#		d: Is a direction to explore
	#	   ob: Is the x,y position of each object along with its radius (obs)
	def lidar_limits_direction(self, x, y, d, ox, oy):
		angle = math.atan2(d[1], d[0])
		rx = x + self.sensor_radius * math.cos(angle)
		ry = y + self.sensor_radius * math.sin(angle)
		r = np.array([rx, ry])
		col, p = self.lidar_limits(x, y, r, ox, oy, "object")
		return col

	# We get the limit for an specific angle
	# Where:
	#	  x,y: Is the position where LIDAR is measuring
	#		r: Is the current limit LIDAR position being explored for object collision
	#	   ob: Is the x,y position of each object along with its radius (obs)
	def lidar_limits(self, x, y, r, ox, oy, mode):
		oi = []
		object_size_standard = self.grid_size/2 + 0.01
		for obx,oby,obs in np.nditer([ox, oy, object_size_standard]):
			if mode == "object":
				# We get the object closest to the LIDAR looking angle.
				robj = np.array([round(r[0]), round(r[1])])
				intersects, limit = self.detect_collision_object(np.array([x, y]), robj, np.array([obx, oby]), obs)
			elif mode == "limit":
				# We get the limit for an specific angle and return collision point.
				intersects, limit = self.detect_collision_point(np.array([x, y]), r, np.array([obx, oby]), obs)
			else:
				logging.error("Not proper mode: " + str(mode))
				exit(-1)
			if (intersects):
				if lidar_intersection_debug:
					logging.debug("lidar_limits() -> Intersection at " + str(limit))
				oi.append(limit)
		if (len(oi) == 0):
			# No collision found
			if lidar_intersection_debug:
				logging.debug("lidar_limits() -> No collision found")
			return False, r
		elif (len(oi) == 1):
			# Found only one collision
			if lidar_intersection_debug:
				logging.debug("lidar_limits() -> Only one collision found | oi[0]: " + str(oi[0]))
			return True, oi[0]
		else:
			# Found multiple collisions, returning closest one
			if lidar_intersection_debug:
				logging.debug("lidar_limits() -> Found multiple collisions: " + str(oi))
			min_val = float("inf")
			val_to_return = None
			for p in oi:
				h = np.linalg.norm(p)
				if lidar_intersection_debug:
					logging.debug("lidar_limits() -> Collisions: " + str(p) + " | min_val: " + str(min_val) + " | h: " + str(h))
				if (h < min_val):
					min_val = h
					val_to_return = p
			if lidar_intersection_debug:
				logging.debug("---------------------------------------------")
			return True, val_to_return

	# We get the limit for an specific angle and return collision point.
	#   Builds a map with limits instead of objects (wich looks bad and leads to errors)
	#   Really useful to find obstacles in the way
	# Where:
	#	   p1: Is the first point of a segment
	#	   p2: Is the second point of a sagment
	#		q: Is the center of a circle that could intersect with the segment
	#		r: Is the radius of the circle that could intersect the segment
	def detect_collision_point(self, p1, p2, q, r):
		intersects, values = self.getSegmentCircleIntersection(p1,p2,q,r)
		# This code returns the proper intersection point
		if (intersects):
			if (len(values) == 1):
				return intersects, values[0]
			else:
				m1 = np.linalg.norm(values[0])
				m2 = np.linalg.norm(values[1])
				if(m1>m2):
					return intersects, values[1]
				else:
					return intersects, values[0]
		else:
			return intersects, None

	# We get the object closest to the LIDAR looking angle.
	#   (Useful to build a map, but leads to errors when finding obstacles in the way)
	# Where:
	#	   p1: Is the first point of a segment
	#	   p2: Is the second point of a segment
	#		q: Is the center of a circle that could intersect with the segment
	#		r: Is the radius of the circle that could intersect the segment
	def detect_collision_object(self, p1, p2, q, r):
		intersects, values = self.getSegmentCircleIntersection(p1,p2,q,r)
		# This code returns the actual objects (which simulation wise is better)
		if (intersects):
			return intersects, q
		else:
			return intersects, None

	def getSegmentCircleIntersection(self, p1, p2, q, r):
		v = np.array([p2[0]-p1[0], p2[1]-p1[1]])
		a = v.dot(v)
		b = 2 * v.dot(np.array([p1[0]-q[0], p1[1]-q[1]]))
		c = p1.dot(p1) + q.dot(q) - 2 * p1.dot(q) - r**2
		disc = b**2 - 4 * a * c
		if disc < 0:
			return False, None
		sqrt_disc = math.sqrt(disc)
		t1 = (-b + sqrt_disc) / (2 * a)
		t2 = (-b - sqrt_disc) / (2 * a)

		if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
			return False, None
		else:
			i = []
			if (0 <= t1 <= 1):
				i1 = v.dot(t1) + p1
				i.append(i1)
			if (0 <= t2 <= 1):
				i2 = v.dot(t2) + p1
				i.append(i2)
			return True, i

	def print(self, msg, mode):
		if mode == "debug":
			logging.debug(msg)
		elif mode == "console":
			print(msg)

import math
import numpy as np
import logging
import json

# START Class LidarPoint ----------------------------------------------
class LidarPoint:
	def __init__(self):
		self.angle = 0
		self.r = np.zeros(2)
		self.dist = 0
		self.col = False

	def print(self):
		logging.info("angle: " + str(math.degrees(self.angle)) + " | oi: " + str(self.r) + " | dist: " + str(self.dist) + " | collision: " + str(self.col))

	def get_coords(self):
		return list(self.r)

	def get_coord_list(self, limits_list):
		oi = []
		for aux in limits_list:
			coords = []
			mylist = aux.get_coords()
			coords.append(int(mylist[0]))
			coords.append(int(mylist[1]))
			oi.append(coords)
		return oi

	def to_dict(self):
		point = {
			"angle": self.angle,
			"rx": self.r[0],
			"ry": self.r[1],
			"dist": self.dist,
			"col": self.col,
			}
		return point

	def from_dict(self, point):
		self.angle = point["angle"]
		self.r = np.zeros(2)
		self.r[0] = point["rx"]
		self.r[1] = point["ry"]
		self.dist = point["dist"]
		self.col = point["col"]
		return self

import math
import numpy as np
import logging

# Modules
from navMap import Map

# START Class Map ------------------------------------------------
class BrushfireNavigation:
	def __init__(self):
		self.reso = None
		self.minx = self.miny = None
		self.maxx = self.maxy = None
		self.xw = self.yw = None
		self.vlimit = None
		self.map = None

	def set_params(self, reso, gx, gy, ox, oy, vlimit):
		self.reso = reso
		self.minx = min(ox)
		self.miny = min(oy)
		self.maxx = max(ox)
		self.maxy = max(oy)
		if gx > self.maxx:
			self.maxx = gx
		if gy > self.maxy:
			self.maxy = gy
		self.xw = int(round((self.maxx - self.minx) / self.reso)) + 1
		self.yw = int(round((self.maxy - self.miny) / self.reso)) + 1
		self.vlimit = vlimit
		self.map = None

	def get_map(self):
		return self.map

	def set_map(self, exmap):
		self.map = exmap

	def get_xw(self):
		return self.xw

	def set_xw(self, xw):
		self.xw = xw

	def get_yw(self):
		return self.yw

	def set_yw(self, yw):
		self.yw = yw

	def show_map(self, mode="debug"):
		self.print("Map--------------------------------------", mode)
		for j in range(self.yw):
			msg = ""
			for i in range(self.xw):
				msg = msg + " " + str('{0:02d}').format(int(self.map[i][int(self.maxy)-j]))
			self.print(msg, mode)
		self.print("-----------------------------------------", mode)

	def update_map(self, myMap, xp, yp, limits):
		# Indices must be integers, not numpy.float64
		xp = int(round(xp,0))
		yp = int(round(yp,0))
		self.print("Brushfire limits -------------------", "debug")
		for p in limits:
			xi = int(round(math.cos(p["angle"])))
			yi = int(round(math.sin(p["angle"])))
			if abs(xi) == abs(yi):
				# We move in diagonal, steps are not the same as distance
				rx = int(round(p["rx"],0))
				ry = int(round(p["ry"],0))
				distx = abs(rx-xp)
				disty = abs(ry-yp)
				if distx > disty:
					steps = distx
				else:
					steps = disty
			else:
				# We move in horizontal or vertical, steps are distance
				steps = int(round(p["dist"],0))
			# Need to adjust this as we start from zero
			steps = steps + 1
			self.print("p: " + str(p) + " | xi: " + str(xi) + " | yi " + str(yi) + " | steps: " + str(steps), "debug")
			# We use the increments to calculate the path to record the "wave"
			for s in range(steps):
				x = xp + xi*s
				y = yp + yi*s
				if (x >= 0) and (y >=0) and (x < self.maxx) and (y < self.maxy):
					new_val = self.get_dist_value(p["col"], steps-s)
					if myMap[x][y] == 0 or myMap[x][y] > new_val:
						myMap[x][y] = new_val
		self.map = myMap
		self.show_map()
		return myMap

	def get_dist_value(self, col, d):
		# There is an object, we trace back and assign distance to object
		if col == True:
			value = d
		# There is no object, we set all to the vision limit
		elif col == False:
			value = self.vlimit
		else:
			# Something went wrong. We shouldn't be here!
			logging.error("We have no collision information!")
			value = -2
		return value

	def print(self, msg, mode):
		if mode == "debug":
			logging.debug(msg)
		elif mode == "console":
			print(msg)

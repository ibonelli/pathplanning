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
		xp = int(xp)
		yp = int(yp)
		#self.print("Brushfire limits -------------------", "console")
		#self.print(str(limits), "console")
		for p in limits:
			# same X position, we increment Y
			if xp == p["rx"]:
				inc_limit = int(abs(p["ry"]-yp))
				for yinc in range(inc_limit):
					if yp+yinc < self.maxy:
						myMap[xp][yp+yinc] = self.get_dist_value(p["col"], inc_limit-yinc)
			# same Y position, we increment X
			elif yp == p["ry"]:
				inc_limit = int(abs(p["rx"]-xp))
				for xinc in range(inc_limit):
					if xp+xinc < self.maxx:
						myMap[xp+xinc][yp] = self.get_dist_value(p["col"], inc_limit-xinc)
			# Diagonal, we increment both
			else:
				inc_limit = int(abs(p["rx"]-xp))
				for inc in range(inc_limit):
					if (xp+inc < self.maxx) and (yp+inc < self.maxy):
						myMap[xp+inc][yp+inc] = self.get_dist_value(p["col"], inc_limit-inc)
		self.map = myMap
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

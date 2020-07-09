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
		self.map = None

	def set_params(self, reso, gx, gy, ox, oy):
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

	#def draw(self):
	#	plt.cla()
	#	for j in range(self.yw):
	#		for i in range(self.xw):
	#			if self.map[i][j] == 1.0:
	#				plt.plot(i*self.reso, j*self.reso, "b.")
	#	plt.axis("equal")
	#	plt.grid(True)
	#	plt.title("Map")
	#	plt.ginput()
	#	plt.savefig(self.debug_map_fname)

	def show_map(self, mode="debug"):
		self.print("Map--------------------------------------", mode)
		for j in range(self.yw):
			msg = ""
			for i in range(self.xw):
				msg = msg + " " + str('{0:02d}').format(int(self.map[i][int(self.maxy)-j]))
			self.print(msg, mode)
		self.print("-----------------------------------------", mode)

	def update_map(self, myMap, xp, yp, limits):
		self.map = myMap
		for j in range(self.yw):
			for i in range(self.xw):
				if self.map[i][j] == 1.0:
					plt.plot(i*self.reso, j*self.reso, "b.")

		return myMap

	def print(self, msg, mode):
		if mode == "debug":
			logging.debug(msg)
		elif mode == "console":
			print(msg)

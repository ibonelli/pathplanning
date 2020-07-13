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
		self.motion = [[1, 0],
					  [0, 1],
					  [-1, 0],
					  [0, -1],
					  [-1, -1],
					  [-1, 1],
					  [1, -1],
					  [1, 1]]

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
		self.print("Map+--------------------------------------", mode)
		msg_xruler = "    "
		for i in range(self.xw):
			msg_xruler = msg_xruler + " " + str('{0:02d}').format(i)
		self.print(msg_xruler, mode)
		self.print("   +----------------------------------------", mode)
		for j in range(self.yw):
			msg = str('{0:02d}').format(int(self.maxy)-j) + " |"
			for i in range(self.xw):
				try:
					msg = msg + " " + str('{0:02d}').format(int(self.map[i][int(self.maxy)-j]))
				except:
					self.print("Error show_map() | maxx : " + str(self.maxx) + " | maxy: " + str(self.maxy), mode)
			msg = msg + " | " + str('{0:02d}').format(int(self.maxy)-j)
			self.print(msg, mode)
		self.print("   +----------------------------------------", mode)
		self.print(msg_xruler, mode)

	def update_map(self, myMap, xp, yp, limits):
		# Indices must be integers, not numpy.float64
		xp = int(round(xp,0))
		yp = int(round(yp,0))
		#self.print("Brushfire limits -------------------", "debug")
		dir_steps = []
		for p in limits:
			# We build direction (as 1s instead of angles)
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
			dir_steps.append((xi,yi,steps,p["col"]))
		dir_steps = sorted(dir_steps, key=lambda mystep: mystep[2])
		for r in dir_steps:
			#self.print("col: " + str(r[3]) + " | xi: " + str(r[0]) + " | yi " + str(r[1]) + " | steps: " + str(r[2]), "debug")
			# We use the increments to calculate the path to record the "wave"
			for s in range(r[2]):
				x = xp + r[0]*s
				y = yp + r[1]*s
				if (x >= 0) and (y >=0) and (x < self.maxx) and (y < self.maxy):
					# There is an object, we trace back and assign distance to object
					if r[3] == True:
						value = r[2]-s
					# There is no object, we set all to the vision limit
					else:
						value = myMap[xp][yp]+s
					if myMap[x][y] == 0 or myMap[x][y] > value:
						# The "myMap[x][y] != 1" evaluation is included as it is the lowest valid val
						myMap[x][y] = value
		self.map = myMap
		#self.show_map()
		return myMap[xp][yp]

	def get_motion_potentials(self, myMap, xp, yp):
		pot = []
		for d in self.motion:
			posx = xp + d[0]
			posy = yp + d[1]
			pot.append((d[0], d[1], myMap[posx][posy]))

		return pot

	def print(self, msg, mode):
		if mode == "debug":
			logging.debug(msg)
		elif mode == "console":
			print(msg)

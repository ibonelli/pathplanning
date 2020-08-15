import math
import numpy as np
import logging

# Modules
import config
from navMap import Map
from navData import NavigationData

# START Class Map ------------------------------------------------
class BrushfireNavigation:
	def __init__(self):
		self.reso = None
		self.minx = self.miny = None
		self.maxx = self.maxy = None
		self.xw = self.yw = None
		self.vlimit = None
		self.map = None
		self.motion = config.general['robot_motion_model']

	def set_params(self, reso, gx, gy, ox, oy, vlimit):
		self.reso = reso
		self.minx = int(round(min(ox),0))
		self.miny = int(round(min(oy),0))
		self.maxx = int(round(max(ox),0))
		self.maxy = int(round(max(oy),0))
		if gx > self.maxx:
			self.maxx = gx
		if gy > self.maxy:
			self.maxy = gy
		self.xw = int(round((self.maxx - self.minx) / self.reso)) + 1
		self.yw = int(round((self.maxy - self.miny) / self.reso)) + 1
		self.vlimit = vlimit
		self.map = None

	def show_limits(self):
		print("Limits:")
		print("\tminx: " + str(self.minx) + " | miny: " + str(self.miny))
		print("\tmaxx: " + str(self.maxx) + " | maxy: " + str(self.maxy))

	def get_limits(self):
		limits = {
			"minx": self.minx,
			"miny": self.miny,
			"maxx": self.maxx,
			"maxy": self.maxy,
			}
		return limits

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

	def show_map(self, xp, yp, mode="debug"):
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
					yi = int(self.maxy)-j
					if i == xp and yi == yp:
						msg = msg + " **"
					else:
						msg = msg + " " + str('{0:02d}').format(int(self.map[i][yi]))
				except:
					self.print("Error show_map() | maxx : " + str(self.maxx) + " | maxy: " + str(self.maxy) + " | i: " + str(i) + " | j: " + str(j), mode)
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

	# From brusfire information it generates indication values.
	# Params:
	# 		* xp, yp : Base point to explore
	#		* rexplore : Radius to explore
	#		* reval : Radius to evaluate around exploration point
	#		* navData : Prior navData object (used to get limit information)
	# Returns:
	# 		* Known points in the area as a percentage.
	#		* Limit size:
	#				- If there is no limit is 0.
	#				- If there is a limit, it returns the sum of connected points.
	def known_areas(self, xp, yp, rexplore, reval, limit, navData):
		# Indices must be integers, not numpy.float64
		xp = int(round(xp,0))
		yp = int(round(yp,0))
		for d in self.motion:
			# Fetching limit information
			gottendata = navData.get_value(d[0], d[1])
			# In case everything fails... We log. But this code is only to avoid uncertain situations.
			if gottendata is None:
				logging.error("gottendata is Empty at: (" + str(xp) + "," + str(yp) + ")")
			else:
				if gottendata['blocked'] is None:
					logging.error("blocked is Empty at: (" + str(xp) + "," + str(yp) + ")")
					logging.error("\t" + str(gottendata))
			# Where to explore
			if gottendata['blocked'] == False:
				posx = xp + d[0] * rexplore
				posy = yp + d[1] * rexplore
				rawval = abs(self.known_point(posx, posy, reval))
				rawconn = 0
			else:
				limx, limy = gottendata['limit_pos']
				#logging.debug("limit_pos: (" + str(limx) + "," + str(limy) + ")")
				rawval = self.known_limit(xp, yp, limx, limy)
				rawconn = self.get_neighbors(limx, limy, limit)

			navdataval = navData.build_info("known", rawval, navData.get_value(d[0], d[1]))
			navData.set_value(d[0], d[1], navdataval)
			navdataval = navData.build_info("block_size", rawconn, navData.get_value(d[0], d[1]))
			navData.set_value(d[0], d[1], navdataval)

		return navData

	def known_point_from_limits(self, xinf, xsup, yinf, ysup):
		rangex = xsup - xinf
		rangey = ysup - yinf
		total = rangex * rangey
		known_points = 0
		if total !=0:
			# Now we evaluate
			for i in range(rangex):
				for j in range(rangey):
					if self.map[xinf+i][yinf+j] != 0:
						known_points+=1
						#if self.map[xinf+i][yinf+j] == 1:
						#	logging.error("NEED TO TAKE INTO ACCOUNT WALLS!!!")
						#	NEW METHOD WITH DIRECTION INFO TO DO THIS
			known = known_points / total
		elif rangex == 1 or rangey == 1:
			known = 1
		else:
			logging.error("Wrong limits for known_point_from_limits()")
			logging.error("\txsup: " + str(xsup) + " | xinf: " + str(xinf) + " | ysup: " + str(ysup) + " | yinf: " + str(yinf))
			known = -1
		#logging.debug("known: " + str(known) + " | known_points: " + str(known_points) + " | total: " + str(total))
		return known

	def known_point(self, xp, yp, radius):
		#logging.debug("known_point()")
		#logging.debug("\txp: " + str(xp) + " | yp: " + str(yp) + " | radius: " + str(radius))
		# We get limits
		xinf = xp - radius
		xsup = xp + radius
		yinf = yp - radius
		ysup = yp + radius
		xinf, yinf, xsup, ysup = self.fix_limits(xinf, yinf, xsup, ysup)
		# We calculate the value and return it
		return self.known_point_from_limits(xinf, xsup, yinf, ysup)

	def known_point_direction(self, xp, yp, dirx, diry, limit):
		logging.debug("known_point_direction()")
		logging.debug("\tpoint: " + str((xp,yp)) + " | dir: " + str((dirx,diry)) + " | limit: " + str(limit))
		# We get arc
		return 0

	def known_limit(self, xp, yp, xl, yl):
		# If we are in vertical or horizontal lines
		if xp == xl:
			yinf = min([yp, yl])
			ysup = max([yp, yl])
			d = int((ysup - yinf)/2)
			xinf = xp - d
			xsup = xp + d
			xinf, yinf, xsup, ysup = self.fix_limits(xinf, yinf, xsup, ysup)
		elif yp == yl:
			xinf = min([xp, xl])
			xsup = max([xp, xl])
			d = int((xsup - xinf)/2)
			yinf = yp - d
			ysup = yp + d
			xinf, yinf, xsup, ysup = self.fix_limits(xinf, yinf, xsup, ysup)
		# We are in a diagonal
		else:
			xinf = min([xp, xl])
			xsup = max([xp, xl])
			yinf = min([yp, yl])
			ysup = max([yp, yl])
			xinf, yinf, xsup, ysup = self.fix_limits(xinf, yinf, xsup, ysup)
		# We calculate the value and return it
		return self.known_point_from_limits(xinf, xsup, yinf, ysup)

	def get_neighbors(self, xl, yl, limit, found=None):
		if limit > 0:
			#logging.debug("get_neighbors() ---------------------------------")
			#logging.debug("\tlimit: " + str(limit))
			neighbors = []
			for xy in self.motion:
				dx, dy = xy
				xp = xl+dx
				yp = yl+dy
				if self.minx <= xp and xp <= self.maxx:
					if self.miny <= yp and yp <= self.maxy:
						if self.map[xp][yp] == 1:
							# We need to discard prior found points
							if found is not None:
								if not(xp == found[0] and yp == found[1]):
									neighbors.append((xl+dx, yl+dy))
							else:
								neighbors.append((xl+dx, yl+dy))
			if found is None:
				# This is not a child!
				# Adding one as even if you don't have neighbors you have the explored point
				count = len(neighbors) + 1
			else:
				count = len(neighbors)
			#logging.debug("\tneighbors: " + str(count))
			childs = 0
			if count > 0:
				for n in neighbors:
					x, y = n
					childs = self.get_neighbors(x, y, limit-1, (xl, yl))
			#logging.debug("\tchilds: " + str(childs))
			#logging.debug("----------------------------- END get_neighbors()")
			to_return = count+childs
		else:
			to_return = 0
		return to_return

	def get_neighbors_list(self, xl, yl, limit, found=None):
		neighbors = []
		siblings = []

		if limit > 0:
			#logging.debug("get_neighbors() ---------------------------------")
			#logging.debug("\tlimit: " + str(limit))
			for xy in self.motion:
				dx, dy = xy
				xp = xl+dx
				yp = yl+dy
				if self.minx <= xp and xp <= self.maxx:
					if self.miny <= yp and yp <= self.maxy:
						if self.map[xp][yp] == 1:
							# We need to discard prior found points
							if found is not None:
								if not(xp == found[0] and yp == found[1]):
									neighbors.append((xl+dx, yl+dy))
							else:
								neighbors.append((xl+dx, yl+dy))
			if found is None:
				# This is not a child!
				# Adding one as even if you don't have neighbors you have the explored point
				count = len(neighbors) + 1
			else:
				count = len(neighbors)
			#logging.debug("\tneighbors: " + str(count))
			if count > 0:
				for n in neighbors:
					x, y = n
					siblings = self.get_neighbors_list(x, y, limit-1, (xl, yl))
			#logging.debug("\tchilds: " + str(childs))
			#logging.debug("----------------------------- END get_neighbors()")

		return neighbors + siblings

	def fix_limits(self, xinf, yinf, xsup, ysup):
		if xinf < self.minx:
			xinf = self.minx
		if yinf < self.miny:
			yinf = self.miny
		if xsup > self.maxx:
			xsup = self.maxx
		if ysup > self.maxy:
			ysup = self.maxy
		return xinf, yinf, xsup, ysup

	def get_motion_potentials(self, xp, yp):
		pot = []
		for d in self.motion:
			posx = xp + d[0]
			posy = yp + d[1]
			pot.append((d[0], d[1], self.map[posx][posy]))
		return pot

	def print(self, msg, mode):
		if mode == "debug":
			logging.debug(msg)
		elif mode == "console":
			print(msg)

	def get_map_params(self):
		mapdata = {
			"reso": self.reso,
			"minx": self.minx,
			"miny": self.miny,
			"maxx": self.maxx,
			"maxy": self.maxy,
			"xw": self.xw,
			"yw": self.yw,
			"vlimit": self.vlimit,
			}
		return mapdata

	def set_map_params(self, mapdata):
		self.reso = mapdata["reso"]
		self.minx = mapdata["minx"]
		self.miny = mapdata["miny"]
		self.maxx = mapdata["maxx"]
		self.maxy = mapdata["maxy"]
		self.xw = mapdata["xw"]
		self.yw = mapdata["yw"]
		self.vlimit = mapdata["vlimit"]

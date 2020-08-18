"""

Potential Field based path planner

Ref:
	https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
Original work by:
	https://github.com/AtsushiSakai/PythonRobotics

"""

import numpy as np
import matplotlib.pyplot as plt
import logging
import time

# Modules
import config
from navMap import Map

class ApfNavigation:
	def __init__(self, reso, rr):
		# Define model constants
		self.KP = 5.0  # attractive potential gain
		self.ETA = 100.0  # repulsive potential gain
		self.motion = config.general['robot_motion_model']
		# Define object properties
		self.reso = reso
		self.rr = rr
		self.pmap = None
		self.minx = None
		self.miny = None
		self.ix = None
		self.iy = None
		self.gix = None
		self.giy = None
		self.scount = None
		self.scount_limit = 3
		self.rcheck = None
		self.stuck = False
		self.curdirx = None
		self.curdiry = None
		self.pvec = []
		self.indecision = False

	def set_cur_pos(self, xp, yp):
		self.ix = xp
		self.iy = yp

	def get_pvec(self):
		return self.pvec

	def calc_potential_field(self, gx, gy, ox, oy):
		myMap = Map()
		myMap.set_params(self.reso, gx, gy, ox, oy)
		pmap = myMap.create()

		for ix in range(myMap.get_xw()):
			x = ix * self.reso + myMap.get_minx()

			for iy in range(myMap.get_yw()):
				y = iy * self.reso + myMap.get_miny()
				ug = self.calc_attractive_potential(x, y, gx, gy)
				uo = self.calc_repulsive_potential(x, y, ox, oy)
				uf = ug + uo
				pmap[ix][iy] = uf

		myMap.set_map(pmap)
		return pmap, myMap.get_minx(), myMap.get_miny()

	def calc_attractive_potential(self, x, y, gx, gy):
		return 0.5 * self.KP * np.hypot(x - gx, y - gy)

	def calc_repulsive_potential(self, x, y, ox, oy):
		# search nearest obstacle
		minid = -1
		minid = -1
		dmin = float("inf")
		for i in range(len(ox)):
			d = np.hypot(x - ox[i], y - oy[i])
			if dmin >= d:
				dmin = d
				minid = i

		# calc repulsive potential
		dq = np.hypot(x - ox[minid], y - oy[minid])

		if dq <= self.rr:
			if dq <= 0.1:
				dq = 0.1

			return 0.5 * self.ETA * (1.0 / dq - 1.0 / self.rr) ** 2
		else:
			return 0.0

	def get_pmap(self):
		return self.pmap

	def get_motion_model(self):
		return self.motion

	def get_cur_dir(self):
		return self.curdirx, self.curdiry

	def set_motion_model(self, motion):
		self.motion = motion

	def reset_motion_model(self):
		self.motion = config.general['robot_motion_model']

	def decide_status_v2(self, rd):
		## Checking if we get stuck...
		dif = rd[-2] - rd[-1]
		# Si aumento la distancia al objetivo
		if dif < 0 and self.indecision == True:
			self.stuck = True
		else:
			self.stuck = False
		return self.stuck

	def decide_status(self, rd):
		## Checking if we get stuck...
		dif = rd[-2] - rd[-1]
		logging.debug("<APF> -- decide_status() | dif: " + str(dif) + " | Indecision: " + str(self.indecision))
		# Si aumento la distancia al objetivo
		if dif < 0 and self.scount == 0:
			self.scount = 1
			self.rcheck = rd[-2]
			logging.debug("<APF> -- Stuck? : " + str(self.rcheck))
		elif self.scount != 0:
			if (self.rcheck - rd[-1]) <= self.reso and self.scount <= self.scount_limit:
				self.scount += 1
				logging.debug("<APF> -- Still stuck... scount: " + str(self.scount))
			elif (self.rcheck - rd[-1]) <= self.reso and self.scount > self.scount_limit:
				logging.debug("<APF> -- Now we are really stuck!!!")
				self.stuck = True
			elif (self.rcheck - rd[-1]) > self.reso:
				logging.debug("<APF> -- We got out of rcheck+reso area")
				self.scount = 0
			else:
				logging.error("<APF> -- How did we get here? (scount!=0)")
		else:
			self.scount = 0

		return self.stuck

	def potential_field_planning(self, sx, sy, gx, gy, ox, oy, start):
		if start:
			print("<APF> -- calc_potential_field() processing has started.")
			t0c=time.clock()
			t0t=time.time()
			self.pmap, self.minx, self.miny = self.calc_potential_field(gx, gy, ox, oy)
			# search path
			d = np.hypot(sx - gx, sy - gy)
			# Search variables
			self.ix = round((sx - self.minx) / self.reso)
			self.iy = round((sy - self.miny) / self.reso)
			self.gix = round((gx - self.minx) / self.reso)
			self.giy = round((gy - self.miny) / self.reso)
			# Results list initialization
			self.rx, self.ry, self.rd = [sx], [sy], [d]
			# Local minimum detection vars
			self.scount = 0
			self.rcheck = None
			self.stuck = False
			t1c = time.clock()
			t1t = time.time()
			msg = "<APF> -- Processing finished. Wall time spent: " + str(round(t1t-t0t,2)) + " | CPU time: " + str(round(t1c-t0c,2))
			logging.debug(msg)
			print(msg)

		motion = self.get_motion_model()

		minp = float("inf")
		self.pvec = []
		local_pvec = []

		self.minix, self.miniy = -1, -1
		for i in range(len(motion)):
			inx = int(self.ix + motion[i][0])
			iny = int(self.iy + motion[i][1])
			if inx >= len(self.pmap) or iny >= len(self.pmap[0]):
				p = float("inf")  # outside area
			else:
				p = self.pmap[inx][iny]
				self.pvec.append((p, motion[i][0], motion[i][1]))
				local_pvec.append((p, (motion[i][0], motion[i][1]), (inx, iny)))

		local_pvec = sorted(local_pvec, key=lambda pvec: pvec[0])

		if local_pvec[0][0] == local_pvec[1][0]:
			# Two directions with same potential, need a better way to decide
			self.indecision = True
		else:
			# Only one best direction, we can go with that one
			self.indecision = False

		self.curdirx = local_pvec[0][1][0]
		self.curdiry = local_pvec[0][1][1]
		self.ix = local_pvec[0][2][0]
		self.iy = local_pvec[0][2][1]

		xp = self.ix * self.reso + self.minx
		yp = self.iy * self.reso + self.miny
		d = np.hypot(gx - xp, gy - yp)

		return d, xp, yp, self.curdirx, self.curdiry

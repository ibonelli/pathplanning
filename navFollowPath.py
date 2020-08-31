import numpy as np
import logging

# Modules
from navMap import Map

# START Class FollowPath --------------------------------------------
class FollowPath:
	def __init__(self, reso, gx, gy):
		self.gx = gx
		self.gy = gy
		self.map = None
		self.reso = reso
		self.limitx = None
		self.limity = None
		self.limit = True

	def set_limit(self, new_gx, new_gy):
		self.limitx = new_gx
		self.limity = new_gy
		self.limit = False

	def get_limit(self):
		return self.limitx, self.limity

	def get_status(self):
		return self.limit

	def follow(self, xp, yp, curdirx, curdiry):
		xs = xp + curdirx * self.reso
		ys = yp + curdiry * self.reso
		d = np.hypot(self.gx - xs, self.gy - ys)
		d_limit = np.hypot(self.limitx - xs, self.limity - ys)

		# Llegamos al objetivo?
		if d_limit < self.reso:
			self.limit = True

		# Verificamos no superar el limite
		if int(self.limitx) == int(xs):
			self.limit = True
		if int(self.limity) == int(ys):
			self.limit = True

		return d, xs, ys, curdirx, curdiry, self.limit

	def follow_steps(self, xp, yp, astar_path):
		idx = astar_path.index((xp, yp))
		if idx == 0:
			logging.error("Already reached goal...")
			logging.error("\tAstar path: " + str(astar_path))
			logging.error("\tCurrent step: " + str((xp,yp)))
		else:
			newx, newy = astar_path[idx-1]

		dirx = newx - xp
		diry = newy - yp
		d = np.hypot(self.gx - newx, self.gy - newy)

		logging.debug("Following Astar steps...")
		logging.error("\tAstar path: " + str(astar_path))
		logging.debug("\txp,yp: " + str((newx, newy)) + " | curdir: " + str((dirx, diry)) + " | d: " + str(d))

		return d, newx, newy, dirx, diry


	def decide_status(self, xp, yp, ox, oy):
		stuck = False
		for obx,oby in np.nditer([ox, oy]):
			if obx == xp and oby == yp:
				stuck = True
		return stuck

	def follow_wall(self, xp, yp, curdirx, curdiry, obsx, obsy):
		# TODO -- Mantenerme siempre a la distancia actual del
		#			path_blocked_dir para evitar nuevo
		#			escenario descripto en WorldBuilder.ods
		# VER el siguiente código
		#	# Bisector angle of two vectors in 2D
		#	# First we get each angle from original directions
		#	theta_u = math.atan2(self.trap_dir[0], self.trap_dir[1])
		#	theta_v = math.atan2(self.dir[-1][0], self.dir[-1][1])
		#	# Then we create a new vector with the average angle
		#	middle_theta = (theta_u+theta_v)/2
		#	m_dir = (math.cos(middle_theta), math.sin(middle_theta))
		xs = xp + curdirx * self.reso
		ys = yp + curdiry * self.reso
		d = np.hypot(self.gx - xs, self.gy - ys)
		d_limit = np.hypot(self.limitx - xs, self.limity - ys)
		if d_limit < self.reso:
			self.limit = True
		return d, xs, ys, curdirx, curdiry, self.limit

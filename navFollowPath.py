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
		if d_limit < self.reso:
			self.limit = True
		return d, xs, ys, curdirx, curdiry, self.limit

	def follow_wall(self, xp, yp):
		xs = xp + curdirx * self.reso
		ys = yp + curdiry * self.reso
		d = np.hypot(self.gx - xs, self.gy - ys)
		d_limit = np.hypot(self.limitx - xs, self.limity - ys)
		if d_limit < self.reso:
			self.limit = True
		return d, xs, ys, curdirx, curdiry, self.limit

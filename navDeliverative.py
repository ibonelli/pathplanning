import logging

# Modules
from navMap import Map

# START Class DelivNavigation --------------------------------------------
class DeliverativeNavigation:
	def __init__(self, radius, gx, gy):
		self.stuck = False
		self.path_blocked = False
		self.org_gx = gx
		self.org_gy = gy
		self.map = None
		self.path = []
		self.dir = []

	def set_map(self, Map):
		self.map = Map

	def get_map(self):
		return self.map

	def set_step(self, step, direction):
		self.path.append(step)
		self.dir.append(direction)

	def new_goal():
		return 0

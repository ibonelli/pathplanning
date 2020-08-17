import logging
import math
import json

ordered_list = [
	(2,1), # 0
	(2,2), # 45
	(1,2), # 90
	(0,2), # 135
	(0,1), # 180
	(0,0), # 225
	(1,0), # 270
	(2,0), # 315
	]

ordered_navigation = [
	(0,1), # 0
	(1,1), # 45
	(1,0), # 90
	(1,-1), # 135
	(0,-1), # 180
	(-1,-1), # 225
	(-1,0), # 270
	(-1,1), # 315
	]

# START Class Map ------------------------------------------------
class NavigationData:
	def __init__(self):
		self.size = 3
		self.values = [[None for j in range(self.size)] for i in range(self.size)]

	def get_data(self):
		return self.values

	def save_data(self, filename):
		data = {
			"values": self.values,
			"size": self.size,
			}
		data2save = json.JSONEncoder().encode(data)
		with open(filename, 'w') as json_file:
			json.dump(data2save, json_file)
		json_file.close()

	def load_data(self, filename):
		with open(filename) as json_data:
			filedata = json.load(json_data)
		data2load = json.JSONDecoder().decode(filedata)
		self.values = data2load["values"]
		self.size = data2load["size"]

	def get_value(self, x, y):
		i, j = self.dir2self((x, y))
		if i < self.size and j < self.size and not(i==1 and j==1):
			return self.values[i][j]
		else:
			return None

	def set_value(self, x, y, value):
		i, j = self.dir2self((x, y))
		if i < self.size and j < self.size and not(i==1 and j==1):
			self.values[i][j] = value
		else:
			logging.error("Bad navData x,y values: " + str((x, y)))

	def get_iterative(self):
		return ordered_navigation

	def build_info(self, datafield, data, previous=None):
		if previous == None:
			previous = {
				"blocked": None,
				"limit_pos": None,
				"pmap": None,
				"known": None,
				"known_dir": None,
				"block_size": None,
				}
		if datafield == "blocked":
			previous["blocked"] = data
		if datafield == "limit_pos":
			previous["limit_pos"] = data
		if datafield == "pmap":
			previous["pmap"] = data
		if datafield == "known":
			previous["known"] = data
		if datafield == "known_dir":
			previous["known_dir"] = data
		if datafield == "block_size":
			previous["block_size"] = data
		return previous

	def print(self):
		logging.debug("navData:")
		#for j in range(self.size):
		#	for i in range(self.size):
		#		if not(i==1 and j==1):
		#			logging.debug("\tx: " + str(i-1) + " | y: " + str(j-1) + " | dir: " + str(math.degrees(self.pangles(math.atan2(j-1,i-1)))) + " | data: " + str(self.values[i][j]))
		for i,j in ordered_list:
			logging.debug("\tx: " + str(i-1) + " | y: " + str(j-1) + " | dir: " + str(math.degrees(self.pangles(math.atan2(j-1,i-1)))) + " | data: " + str(self.values[i][j]))

	# Using positive angles is more human readable
	def pangles(self, ang):
		if ang >= 0:
			return ang
		else:
			return (2 * math.pi + ang)

	# Internally we go from 0 to 2, but values go from -1 to 1
	def dir2self(self, dirval):
		return dirval[0]+1, dirval[1]+1

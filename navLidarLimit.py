import math
import numpy as np
import matplotlib.pyplot as plt
import logging
import json
from navLidarPoint import LidarPoint

# START Class navGrafLimit ----------------------------------------------
class LidarLimit:
	def __init__(self):
		self.debug_limit_fname = "limit.png"

	# From the LIDAR list we get obstacles
	def get_limits(self, limit):
		oi_list = []

		for l in limit:
			if (l.col == True):
				oi_list.append(l)

		return oi_list

	# From the LIDAR list we get best possible paths
	def get_freepath(self, limit):
		fp_list = []

		for l in limit:
			if (l.col == False):
				fp_list.append(l)

		return fp_list

	# Limits to map
	def limit2map(self, omap, limit):
		#limit = self.get_limits(path_limit)
		if (len(limit) >= 1):
			for l in limit:
				ox = int(l[0])
				oy = int(l[1])
				omap[ox][oy] = 1.0
		return omap

	# From the LIDAR list we get obstacles windows
	def get_limit_windows(self, limit, cX, cY):
		windows = []
		cur_window = PathWindow()
		total = len(limit)
		#logging.debug("get_limit_windows --- Start")

		if total != 1:
			for i in range(total):
				l = limit[i]
				#logging.debug("limit[" + str(i) + "]:")
				#l.print_values()
				if i == 0:
					# Primer valor de la lista
					cur_window.start = math.atan2(l.r[1] - cY, l.r[0] - cX)
					init_type = cur_window.blocked = l.col
					init_start = cur_window.start
				elif i == total-1:
					# Ultimo valor de la lista
					if init_type == cur_window.blocked:
						# Continuamos en el mismo valor blocked
						if len(windows) == 0:
							cur_window.start = cur_window.end = 0
							windows.append(cur_window)
						else:
							windows[0].start = cur_window.start
					else:
						# Cambiamos de valor blocked
						cur_window.end = init_start
						windows.append(cur_window)
				else:
					# Dentro de la lista
					if cur_window.blocked != l.col:
						ang = math.atan2(l.r[1] - cY, l.r[0] - cX)
						cur_window.end = ang
						windows.append(cur_window)
						cur_window = PathWindow()
						cur_window.start = ang
						cur_window.blocked = l.col
		else:
			logging.error("Only 1 limit")
			exit(-1)

		#logging.debug("get_limit_windows --- End")
		return windows

	# Graph LIDAR limit
	def graph_limits(self, limit):
		oi_list = self.get_limits(limit)
		fp_list = self.get_freepath(limit)
		plt.cla()
		if (len(fp_list) >= 1):
			for l in fp_list:
				plt.plot(l.r[0], l.r[1], "b.")
		if (len(oi_list) >= 1):
			for l in oi_list:
				plt.plot(l.r[0], l.r[1], "xr")
		plt.axis("equal")
		plt.grid(True)
		plt.title("Lidar")
		plt.savefig(self.debug_limit_fname)
		plt.ginput()

	# Graph LIDAR polar limit
	def graph_limits_polar(self, limit):
		plt.cla()
		sensor_angle_steps = len(limit)
		angle_step = limit[1].angle - limit[0].angle
		theta = np.linspace(0.0, 2 * np.pi, sensor_angle_steps, endpoint=False)
		g = []
		b = []
		for l in limit:
			g.append(np.linalg.norm(l.r))
			b.append(l.col)
		radii = np.array(g)
		ax = plt.subplot(111, projection='polar')
		bars = ax.bar(theta, radii, width=angle_step, bottom=0.0)
		# Use one color for block path, and a different one for free path
		for l, bar in zip(b, bars):
			if l == False:
				# Dark Blue (RGB)
				i=(0.1, 0.1, 1.0, 1.0)
			else:
				# Dark Red (RGB)
				i=(1.0, 0.1, 0.1, 1.0)
			bar.set_facecolor(i)
			bar.set_alpha(1.0)
		plt.ginput()

	def save_limit(self, x, y, limit, filename):
		limits_to_json = {
			"x": x,
			"y": y,
			"limit": [],
			}
		for l in limit:
			limits_to_json["limit"].append(l.to_dict())
		limits2save = json.JSONEncoder().encode(limits_to_json)
		with open(filename, 'w') as json_file:
			json.dump(limits2save, json_file)
		json_file.close()

	def load_limit(self, filename):
		limits = []
		with open(filename) as json_data:
			data = json.load(json_data)
		saved_limits = json.JSONDecoder().decode(data)
		for l in saved_limits["limit"]:
			p = LidarPoint()
			limits.append(p.from_dict(l))
		return saved_limits["x"], saved_limits["y"], limits

# START Class PathWindow -----------------------------------------------
class PathWindow:
	def __init__(self):
		self.blocked = False
		self.start = 0
		self.end = 0

	# From the LIDAR list we get obstacles windows
	def print(self, mode="debug"):
		msg = "Blocked: " + str(self.blocked) + " | Start: " + str(math.degrees(self.pangles(self.start))) + " | End: " + str(math.degrees(self.pangles(self.end)))
		if mode == "debug":
			logging.debug(msg)
		else:
			print(msg)

	# For the logic to work we need to have positive angles
	# But atan2() returns negatives as well
	def pangles(self,ang):
		if ang >= 0:
			return ang
		else:
			return (2 * math.pi + ang)

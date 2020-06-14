import numpy as np
import matplotlib.pyplot as plt
import json

class ShowNavigation:
	def __init__(self):
		plt.cla()
		plt.grid(True)
		plt.axis("equal")
		self.heatmap = None
		self.start = None
		self.goal = None
		self.path = []

	def draw_heatmap(self, heatmap):
		self.heatmap = heatmap
		data = np.array(self.heatmap).T
		plt.pcolor(data, vmax=200.0, cmap=plt.cm.Blues)

	def show(self, sx, sy, gx, gy):
		self.start = [sx, sy]
		plt.plot(sx, sy, "*k")
		self.goal = [gx, gy]
		plt.plot(gx, gy, "*m")

	def step(self, xp, yp, delay):
		self.path.append([xp,yp])
		plt.plot(xp, yp, ".r")
		plt.pause(delay)

	def save(self, filename):
		navdata = {
			"heatmap": self.heatmap,
			"start": self.start,
			"goal": self.goal,
			"path": self.path,
			}
		nav2save = json.JSONEncoder().encode(navdata)
		with open(filename, 'w') as json_file:
			json.dump(nav2save, json_file)
		json_file.close()

	def load(self, filename):
		with open(filename) as json_data:
			data = json.load(json_data)
		navdata = json.JSONDecoder().decode(data)
		self.heatmap = navdata["heatmap"]
		self.start = navdata["start"]
		self.goal = navdata["goal"]
		self.path = navdata["path"]

	def draw_saved_data(self, delay):
		data = np.array(self.heatmap).T
		plt.pcolor(data, vmax=200.0, cmap=plt.cm.Blues)
		plt.plot(self.start[0], self.start[1], "*k")
		plt.plot(self.goal[0], self.goal[1], "*m")
		for p in self.path:
			plt.plot(p[0], p[1], ".r")
			plt.pause(delay)
		plt.show()

import matplotlib.pyplot as plt
import json

# START Class Map ------------------------------------------------
class Map:
	def __init__(self):
		self.reso = None
		self.minx = self.miny = None
		self.maxx = self.maxy = None
		self.xw = self.yw = None
		self.map = None

	def set_params(self, reso, gx, gy, ox, oy):
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
		self.map = None

	def create(self):
		# calc each potential
		self.map = [[0 for j in range(self.yw)] for i in range(self.xw)]
		return self.map

	def get_map(self):
		return self.map

	def set_map(self, exmap):
		self.map = exmap

	def save_map(self, filename):
		mapdata = {
			"reso": self.reso,
			"minx": self.minx,
			"miny": self.miny,
			"maxx": self.maxx,
			"maxy": self.maxy,
			"xw": self.xw,
			"yw": self.yw,
			"map": self.map,
			}
		map2save = json.JSONEncoder().encode(mapdata)
		with open(filename, 'w') as json_file:
			json.dump(map2save, json_file)
		json_file.close()

	def load_map(self, filename):
		with open(filename) as json_data:
			data = json.load(json_data)
		mapdata = json.JSONDecoder().decode(data)
		self.reso = mapdata["reso"]
		self.minx = mapdata["minx"]
		self.miny = mapdata["miny"]
		self.maxx = mapdata["maxx"]
		self.maxy = mapdata["maxy"]
		self.xw = mapdata["xw"]
		self.yw = mapdata["yw"]
		self.map = mapdata["map"]

	def get_minx(self):
		return self.minx

	def set_minx(self, minx):
		self.minx = minx

	def get_miny(self):
		return self.miny

	def set_miny(self, miny):
		self.miny = miny

	def get_maxx(self):
		return self.maxx

	def set_maxx(self, maxx):
		self.maxx = maxx

	def get_maxy(self):
		return self.maxy

	def set_maxy(self, maxy):
		self.maxy = maxy

	def get_xw(self):
		return self.xw

	def set_xw(self, xw):
		self.xw = xw

	def get_yw(self):
		return self.yw

	def set_yw(self, yw):
		self.yw = yw

	def draw(self, fname):
		plt.cla()
		for j in range(self.yw):
			for i in range(self.xw):
				if self.map[i][j] == 1.0:
					plt.plot(i*self.reso, j*self.reso, "b.")
		plt.axis("equal")
		plt.grid(True)
		plt.title("Map")
		plt.ginput()
		plt.savefig(fname)

	def get_objects(self):
		ox = []
		oy = []
		for j in range(self.yw):
			for i in range(self.xw):
				if self.map[i][j] == 1.0:
					ox.append(self.reso*i)
					oy.append(self.reso*j)
		return ox,oy

	def get_map_params(self):
		mapdata = {
			"reso": self.reso,
			"minx": self.minx,
			"miny": self.miny,
			"maxx": self.maxx,
			"maxy": self.maxy,
			"xw": self.xw,
			"yw": self.yw,
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

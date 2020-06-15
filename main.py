"""

Potential Field based path planner

Ref:
	https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import logging

# Modules
import config
from navApfNavigation import ApfNavigation
from navLidar import Lidar
from navLidarPoint import LidarPoint
from navTrapNavigation import TrapNavigation
from navMap import Map
from navGraf import ShowNavigation
from navLidarLimit import LidarLimit

show_animation = config.general['animation']
graf_delay = config.general['grafDelay']
logging.basicConfig(filename=config.general['logFile'],level=config.general['logLevel'])

# START Class MapNewPath --------------------------------------------
class MapNewPath:
	def __init__(self, reso, rr, radius):
		self.reso = reso
		self.rr = rr
		self.vision_limit = radius
		self.angle = math.atan(rr / radius)
		self.angle_step = int(2 * math.pi / self.angle)
		self.motion = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]]
# END Class MapNewPath --------------------------------------------------

def main():
	# Cargando el archivo de descripcion del mundo a recorrer
	if(len(sys.argv) < 2):
		print('Ingresar el mundo a recorrer. Por ejemplo world1.csv')
		exit(-1)
	else:
		fname = sys.argv[1]

	print("potential_field_planning start")

	grid_size = 1  # potential grid size [m]
	robot_radius = 5.0  # robot radius [m]
	vision_limit = 15

	ob = np.loadtxt(fname)
	flx = ob[:, 0]  # file x position list [m]
	fly = ob[:, 1]  # file y position list [m]
	sx = flx[0]	 # start x position [m]
	sy = fly[0]	 # start y positon [m]
	gx = flx[1]	 # goal x position [m]
	gy = fly[1]	 # goal y position [m]
	ox = flx[2:]   # obstacle x position list [m]
	oy = fly[2:]   # obstacle y position list [m]

	if show_animation:
		MyGraf = ShowNavigation()

	# Navigation Starting
	myNavigation = ApfNavigation(grid_size, robot_radius)
	trap = TrapNavigation(grid_size, robot_radius, vision_limit)

	# Map generation
	myMap = Map()
	myMap.set_params(grid_size, gx, gy, ox, oy)
	myMap.create()
	myLidar = Lidar(grid_size, vision_limit, 8)
	limits = myLidar.fetch_limits(sx, sy, ox, oy, "object")
	myLimits = LidarLimit()
	myMap.set_map(myLimits.limit2map(myMap.get_map(), limits))

	# Navigation 1st step
	stuck = False
	d = float(np.hypot(sx - gx, sy - gy))
	rd, rx, ry = [d], [sx], [sy]
	d, xp, yp, curdirx, curdiry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, True)
	rd.append(d)
	rx.append(xp)
	ry.append(yp)
	if show_animation:
		MyGraf.draw_heatmap(myNavigation.get_pmap())
		MyGraf.show(sx, sy, gx, gy)
		MyGraf.step(xp, yp, graf_delay)

	# Map update and trap detection
	limits = myLidar.fetch_limits(sx, sy, ox, oy, "object")
	myMap.set_map(myLimits.limit2map(myMap.get_map(), limits))
	path_blocked = trap.detect(myMap, sx, sy, curdirx, curdiry)

	# Main navigation loop
	while d >= grid_size and not stuck:
		d, xp, yp, curdirx, curdiry = myNavigation.potential_field_planning(sx, sy, gx, gy, ox, oy, False)
		rd.append(d)
		rx.append(xp)
		ry.append(yp)
		if show_animation:
			MyGraf.step(xp, yp, graf_delay)

		stuck = myNavigation.decide_status(rd)
		limits = myLidar.fetch_limits(xp, yp, ox, oy, "object")
		myMap.set_map(myLimits.limit2map(myMap.get_map(), limits))
		path_blocked = trap.detect(myMap, xp, yp, curdirx, curdiry)

		#This blocks by path_blocked
		#if path_blocked and not stuck:
		if stuck or path_blocked:
			motion_model = trap.propose_motion_model(myMap, xp, yp)
			if len(motion_model) < 1:
				print("We can no longer navigate")
				logging.debug("We can no longer navigate")
				logging.debug("motion model: " + str(motion_model))
				myMap.draw()
				checkMyLimits = Lidar(grid_size, vision_limit, 36)
				limits = checkMyLimits.lidar(xp, yp, ox, oy, "limit")
				debug_graph_fname = "navigation.png"
				plt.savefig(debug_graph_fname)
				myLimits.graph_limits(limits)
				myLimits.graph_limits_polar(limits)
				myMap.save_map("map.json")
				MyGraf.save("path.json")
				myLimits.save_limit(xp, yp, limits, "limit.json")
				exit(0)
			myNavigation.set_motion_model(motion_model)
			#org_gx = gx
			#org_gy = gy
			#gx =
			#gy =
			stuck = False
		else:
			motion_model = trap.reset_motion_model()

	myMap.save_map("map.json")
	MyGraf.save("path.json")

	print("Goal!!")

	if show_animation:
		# We save to a file
		base=os.path.basename(fname)
		plt.savefig(os.path.splitext(base)[0] + "_nav.png")
		plt.show()


if __name__ == '__main__':
	print(__file__ + " start!!")
	main()
	print(__file__ + " Done!!")

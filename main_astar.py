import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

# Modules
import config
from navAstar import AStarPlanner

grid_size = config.general['grid_size']
robot_radius = config.general['robot_radius']
saveResults = config.general['saveResults']
saveReport = config.general['saveReport']
showGrafs = config.general['showGrafs']

def main():
	# Cargando el archivo de descripcion del mundo a recorrer
	if(len(sys.argv) < 2):
		print('Ingresar el mundo a recorrer. Por ejemplo world1.csv')
		exit(-1)
	else:
		fname = sys.argv[1]

	print("potential_field_planning start")

	ob = np.loadtxt(fname)
	flx = ob[:, 0]  # file x position list [m]
	fly = ob[:, 1]  # file y position list [m]
	sx = int(flx[0])	 # start x position [m]
	sy = int(fly[0])	 # start y positon [m]
	gx = int(flx[1])	 # goal x position [m]
	gy = int(fly[1])	 # goal y position [m]
	ox_numpy = flx[2:]   # obstacle x position list [m]
	oy_numpy = fly[2:]   # obstacle y position list [m]

	ox = []
	for oxi in ox_numpy:
		ox.append(int(oxi))

	oy = []
	for oyi in oy_numpy:
		oy.append(int(oyi))

	# Preparing Animation
	plt.cla()
	if not showGrafs:
		plt.ioff()
	plt.plot(ox, oy, ".k")
	plt.plot(sx, sy, "og")
	plt.plot(gx, gy, "xb")
	plt.grid(True)
	plt.axis("equal")

	# Navigation Planning
	a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
	rx, ry, newgx, newgy, known = a_star.planning(sx, sy, gx, gy)

	# Showing Animation
	plt.plot(rx, ry, "-r")
	if showGrafs:
		plt.show()

	if newgx == gx and newgy == gy:
		goal_reached = True
		steps_to_goal = len(rx)
		print("Goal!!")

	if saveResults:
		# We save navigation data files
		fbase=fname[0:-4] # We cut the extension
		plt.savefig(fbase + "_astar_nav.png")

	if saveReport:
		fbase=fname[0:-4] # We cut the extension
		filename = fbase + "_astar_report.txt"
		freport = open(filename, 'w')
		freport.write("World: " + str(fbase) + "\n")
		freport.write("Method: Astar\n")
		if goal_reached:
			freport.write("Result: Found Goal\n")
		else:
			freport.write("Result: Failed\n")
		freport.write("Steps: " + str(steps_to_goal) + "\n\n")
		freport.close()

if __name__ == '__main__':
	print(__file__ + " start!!")
	main()
	print(__file__ + " Done!!")

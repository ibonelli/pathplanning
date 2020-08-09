"""

Tool to:
	1. Show world from CSV Spreadsheet built map
	2. Convert it to a CSV to use for planning

It takes:
	1. file_goal.csv tiene dos lineas con start y goal points
	2. file_map.csv creado con WorldBuilder.ods (tiene 1s donde hay obstaculos)
Outputs file.csv:
	1. Con dos lineas para start y goal points
	2. Una linea con las coordenadas de cada objeto encontrado en file_map.csv

"""

import os
import sys
import numpy as np
	# https://docs.scipy.org/doc/numpy/user/basics.creation.html
import matplotlib.pyplot as plt
	# https://matplotlib.org/users/pyplot_tutorial.html

def main():
	# Cargando el archivo de descripcion del mundo a recorrer
	if(len(sys.argv) < 2):
		print('Ingresar el mundo creado. Por ejemplo: world1')
		exit(-1)
	else:
		fname = sys.argv[1]

	# obstacles [ob1(x,y,r), ob2(x,y,r), ....]
	# x,y coord and obstacle radius
	coords = np.loadtxt(fname + str("_goal.csv"))
	csv = np.loadtxt(fname + str("_map.csv"), delimiter=",")
	obs = 1.0

	# goal position [x, y]
	start = np.array([coords[0,0], coords[0,1]])
	goal = np.array([coords[1,0], coords[1,1]])

	output = np.array([coords[0,0], coords[0,1]])
	output = np.vstack([output, goal])

	ox = []  # obstacle x position list [m]
	oy = []  # obstacle y position list [m]

	ly,lx = csv.shape
	#print("lx: " + str(lx) + " | ly: " + str(ly))

	for iy,ix in np.ndindex(csv.shape):
		#print("ix: " + str(ix) + " | iy: " + str(iy))
		if int(csv[iy,ix]) == 1:
			ox.append(ix)
			oy.append(ly-iy-1)   # Las coordenadas arrancan en la esquina izquierda
			output = np.vstack([output, np.array([ix, ly-iy-1])])

	# Save converted file
	np.savetxt(fname + str(".csv"), output, fmt='%d',)

	plt.cla()
	plt.plot(start[0], start[1], "xg")
	plt.plot(goal[0], goal[1], "ob")
	# ob[:, 0] -> The full first row of the array (all X numbers)
	# ob[:, 1] -> The full second row of the array (all Y numbers)
	#plt.plot(ob[:, 0], ob[:, 1], "ok")
	for obx,oby in np.nditer([ox, oy]):
		patch=plt.Circle((obx, oby), obs, color='black', fill=True)
		tmp=plt.gca()
		tmp.add_patch(patch)
	plt.axis("equal")
	plt.grid(True)

	# We save to a file
	base=os.path.basename(fname)
	plt.savefig(os.path.splitext(base)[0] + ".png")
	plt.show()

if __name__ == '__main__':
	main()

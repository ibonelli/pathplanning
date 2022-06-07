"""

Tool to show world from CSV show_world created map
	The CSV already has start, destination and obstacle list.
It takes worldxx.csv file and shows the world image.

"""

import os
import sys
import numpy as np
	# https://docs.scipy.org/doc/numpy/user/basics.creation.html
import matplotlib.pyplot as plt
	# https://matplotlib.org/users/pyplot_tutorial.html

def main():
	# Cargando el archivo de descripcion del mundo a mostrar
	if(len(sys.argv) < 2):
		print('Ingresar el mundo creado. Por ejemplo: world1.csv')
		exit(-1)
	else:
		fname = sys.argv[1]

	ob = np.loadtxt(fname)
	flx = ob[:, 0]  # file x position list [m]
	fly = ob[:, 1]  # file y position list [m]
	sx = flx[0]	 # start x position [m]
	sy = fly[0]	 # start y positon [m]
	gx = flx[1]	 # goal x position [m]
	gy = fly[1]	 # goal y position [m]
	ox = flx[2:]   # obstacle x position list [m]
	oy = fly[2:]   # obstacle y position list [m]

	plt.cla()
	plt.plot(sx, sy, "xg")
	plt.plot(gx, gy, "ob")
	for obx,oby in np.nditer([ox, oy]):
		patch=plt.Circle((obx, oby), 1.0, color='black', fill=True)
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

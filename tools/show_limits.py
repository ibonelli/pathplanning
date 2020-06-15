"""

Tool to:
	Show a JSON encoded map

"""

import sys

# We need a module from parent folder
sys.path.append('../')

# Modules
from navLidarPoint import LidarPoint
from navLidar import Lidar, PathWindow

grid_size = 1  # potential grid size [m]
vision_limit = 15

def main():
	# Cargando el archivo de descripcion del mundo a recorrer
	if(len(sys.argv) < 2):
		print('Ingresar el limit a mostrar. Por ejemplo: limit.json')
		exit(-1)
	else:
		fname = sys.argv[1]

	checkMyLimits = Lidar(grid_size, vision_limit, 36)

	x, y, limits = checkMyLimits.load_limit(fname)
	myMap.draw()

if __name__ == '__main__':
	main()

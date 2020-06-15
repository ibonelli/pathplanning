"""

Tool to:
	Show a JSON encoded map

"""

import sys

# We need a module from parent folder
sys.path.append('../')

# Modules
from navLidarPoint import LidarPoint
from navLidarLimit import LidarLimit

def main():
	# Cargando el archivo de descripcion del mundo a recorrer
	if(len(sys.argv) < 2):
		print('Ingresar el limit a mostrar. Por ejemplo: limit.json')
		exit(-1)
	else:
		fname = sys.argv[1]

	myLimits = LidarLimit()
	x, y, limits = myLimits.load_limit(fname)
	myLimits.graph_limits(limits)
	myLimits.graph_limits_polar(limits)

if __name__ == '__main__':
	main()

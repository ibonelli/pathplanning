"""

Tool to:
	Show a JSON encoded path

"""

import sys

# We need a module from parent folder
sys.path.append('../')

# Modules
from navGraf import ShowNavigation

grafDelay = 0.01

def main():
	# Cargando el archivo de descripcion del mundo a recorrer
	if(len(sys.argv) < 2):
		print('Ingresar el mapa a mostrar. Por ejemplo: map.json')
		exit(-1)
	else:
		fname = sys.argv[1]

	MyGraf = ShowNavigation()
	MyGraf.load(fname)
	MyGraf.draw_saved_data(grafDelay)

if __name__ == '__main__':
	main()

"""

Tool to:
    Show a JSON encoded map

"""

import sys
import time

# We need a module from parent folder
sys.path.append('../')

# Modules
from navMap import Map

def main():
    # Cargando el archivo de descripcion del mundo a recorrer
    if(len(sys.argv) < 2):
        print('Ingresar el mapa a mostrar. Por ejemplo: map.json')
        exit(-1)
    else:
        fname = sys.argv[1]

    myMap = Map()
    myMap.load_map(fname)
    myMap.draw()

if __name__ == '__main__':
    main()
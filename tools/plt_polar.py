# Original code from:
#		https://matplotlib.org/examples/pie_and_polar_charts/polar_bar_demo.html

import numpy as np
import matplotlib.pyplot as plt

# Compute pie slices
N = 8
theta = np.linspace(0.0, 2 * np.pi, N, endpoint=False)
radii = 10 * np.random.rand(N)
width = 2 * np.pi / N

ax = plt.subplot(111, projection='polar')
bars = ax.bar(theta, radii, width=width, bottom=0.0)

plt.show()
 

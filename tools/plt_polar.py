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

# Use two colors and opacity
flag = False
for r, bar in zip(radii, bars):
	if flag == False:
		i=0.2
		flag = True
	else:
		i=0.8
		flag = False
	bar.set_facecolor(plt.cm.viridis(i))
	bar.set_alpha(0.7)

plt.show()
 

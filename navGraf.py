import numpy as np
import matplotlib.pyplot as plt

class ShowNavigation:
    def __init__(self):
        plt.grid(True)
        plt.axis("equal")

    def draw_heatmap(self, heatmap):
        data = np.array(heatmap).T
        plt.pcolor(data, vmax=200.0, cmap=plt.cm.Blues)

    def show(self, sx, sy, gx, gy):
        plt.plot(sx, sy, "*k")
        plt.plot(gx, gy, "*m")

    def step(self, xp, yp):
        plt.plot(xp, yp, ".r")
        plt.pause(0.01)

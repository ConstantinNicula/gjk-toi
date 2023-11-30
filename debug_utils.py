import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.patches import Polygon 

class DebugPlotter: 
    def __init__(self, x_bounds=None, y_bounds=None):
        self.fig, self.ax = plt.subplots()
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.polygons = []
        self.polygon_colors = []

    def add_poly(self, verts, color): 
        self.polygons.append(verts)  
        # todo chose random color  
        self.polygon_colors.append(color)

    def display(self): 
        if not self.x_bounds:
            self.x_bounds = self.__compute_limits(0)
        self.ax.set_xlim(self.x_bounds)

        if not self.y_bounds:
            self.y_bounds = self.__compute_limits(1)
        self.ax.set_ylim(self.y_bounds)

        plt.grid()
        
        for poly_verts, poly_color in zip(self.polygons, self.polygon_colors):
            if len(poly_verts) > 2:
                p = Polygon(poly_verts, facecolor=poly_color, alpha=0.2)
                self.ax.add_patch(p)

            #draw points 
            xs, ys = zip(*poly_verts)
            plt.scatter(xs, ys)

            # draw contour
            xs, ys = xs+(xs[0],), ys+(ys[0],)
            plt.plot(xs, ys)
        plt.axis('equal')
        plt.show()

    def __compute_limits(self, axis=0, padding=1):
        # stack all points and find min max 
        all_pts = np.vstack(self.polygons)
        min, max = np.min(all_pts[:, axis]), np.max(all_pts[:, axis])
        min = np.floor(min) - padding
        max = np.ceil(max) + padding
        print(min, max) 
        return min, max 



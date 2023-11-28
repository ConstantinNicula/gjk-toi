import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.patches import Polygon 

class Plotter: 
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
            p = Polygon(poly_verts, facecolor=poly_color, alpha=0.2)
            self.ax.add_patch(p)

            #draw points 
            xs, ys = zip(*poly_verts)
            plt.scatter(xs, ys)

            # draw contour
            xs, ys = xs+(xs[0],), ys+(ys[0],)
            plt.plot(xs, ys)

        plt.show()

    def __compute_limits(self, axis=0, padding=1):
        # stack all points and find min max 
        all_pts = np.vstack(self.polygons)
        min, max = np.min(all_pts[:, axis]), np.max(all_pts[:, axis])
        min = np.floor(min) - padding
        max = np.ceil(max) + padding
        print(min, max) 
        return min, max 


class Minkowski: 
    ''' Compute A - B Minkowski difference/sum:
            - take all points a_i in A and subtract all points b_j in B
            - take convex hull of result
    '''
    @staticmethod    
    def brute_force_compute(mesh1, mesh2, sum=False):
        minkowski_mesh = None  
        if sum: 
            minkowski_mesh = np.repeat(mesh1, mesh2.shape[0], 0) + np.tile(mesh2, (mesh1.shape[0], 1))
        else:
            minkowski_mesh = np.repeat(mesh1, mesh2.shape[0], 0) - np.tile(mesh2, (mesh1.shape[0], 1))

        return Minkowski.compute_hull(minkowski_mesh)
         
    
    @staticmethod
    def compute_hull(verts):
        # starting point farthest point left towards -x 
        Hi = [np.argmin(verts[:, 0])]
        while True:
            h_st = Hi[-1]; # last point on hull is start of new seg
            h_end = (h_st + 1) % len(verts) 
            for h_c in range(0, len(verts)):
                if h_c == h_st or h_c == h_end:
                    continue
                # check if h_c is to the left hand side of segment[h_st, h_end]
                if Minkowski.compute_cross(verts[h_st], verts[h_end], verts[h_c]) < 0: 
                    h_end = h_c
            if h_end == Hi[0]:
                break;               
            Hi.append(h_end)
        return verts[Hi]


    @staticmethod
    def compute_cross(p_st, p_end, p_c): 
        return (p_end[0] - p_st[0])*(p_c[1] - p_st[1])  \
                - (p_end[1] - p_st[1])*(p_c[0] - p_st[0])              

def main():
    p1 = np.array([ [1, 1], [2, 1], [2, 2], [1, 2], [0.5, 1.5]])
    p2 = np.array([ [2, 3], [3, 1], [1, 1]])
    p3 = np.array([ [5, 5], [7, 7]])

    plotter = Plotter()
    plotter.add_poly(p1, "k")
    plotter.add_poly(p2, "k")
    #plotter.add_poly(p3, "r")
    plotter.add_poly(Minkowski.brute_force_compute(p2, p1), "b")
    #plotter.add_poly(Minkowski.brute_force_compute(p2, p1, sum=True), "b")
    plotter.display()


if __name__ == "__main__":
    main()


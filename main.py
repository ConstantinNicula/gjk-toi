import numpy as np 
from debug_utils import DebugPlotter
from minkowski import MinkowskiMesh2D

def main():
    p1 = np.array([ [1, 1], [2, 1], [2, 2], [1, 2], [0.5, 1.5]])
    p2 = np.array([ [2, 3], [3, 1], [1, 1]])
    p3 = np.array([ [5, 5], [7, 7]])

    plotter = DebugPlotter()
    plotter.add_poly(p1, "k")
    plotter.add_poly(p2, "k")
    #plotter.add_poly(p3, "r")

    mdiff = MinkowskiMesh2D(p2, p1)
    plotter.add_poly(mdiff.mesh, "b")
    plotter.display()


if __name__ == "__main__":
    main()


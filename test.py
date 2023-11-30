import numpy as np 
from debug_utils import DebugPlotter
from minkowski import MinkowskiMesh2D
from simplex import Simplex
import time 
import random
import glm_utils

def test_minkowski():
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

def random_2d_point():
    return [random.uniform(-2, 2), random.uniform(-2, 2)] 

def test_simplex():
    simplex_verts = [[0, 0], [1, 0], [1, 1]]
    num_query_points = 20
    query_points = [ random_2d_point() for i in range(num_query_points)]

    start_time = time.process_time_ns()
    closest_points = []
    for point in glm_utils.to_glm_vec_list(query_points):
        # create simplex based on points 
        simplex = Simplex()
        for vert in glm_utils.to_glm_vec_list(simplex_verts):
            simplex.add_point(vert, (0, 0)) 

        # find closest point and store 
        closest_points.append(simplex.find_closest_point_on_simplex(point))
    delta_time = time.process_time_ns() - start_time
    print(f"elapsed time {delta_time}ns")

    plotter = DebugPlotter()
    plotter.add_poly(simplex_verts, "r")
    for p, q in zip(query_points, closest_points):
        plotter.add_poly([p, [q[0], q[1]]], "k")
    plotter.display()
    print(closest_points)

if __name__ == "__main__":
    #test_minkowski()
    test_simplex()

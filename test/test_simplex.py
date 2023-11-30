import glm
import sys 

# import own libs
sys.path.append('..')
from simplex import Simplex
from debug_utils import DebugPlotter


def main():
    simplex = Simplex()
    simplex.add_point(glm.vec3(0, 0, 0), (0, 0))   
    simplex.add_point(glm.vec3(1, 0, 0), (0, 0))   
    simplex.add_point(glm.vec3(1, 1, 0), (0, 0))   
    print(simplex)

    q = simplex.find_closest_point_on_simplex(glm.vec3(2, 3, 0))
    print(q)

if __name__ == "__main__":
    main()
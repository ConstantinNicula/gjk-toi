import glm
from collections import namedtuple
from simplex import Simplex
from physics_object import PhysicsObject

CollisionData = namedtuple("CollisionData", ["hit", "simplex", "closest_points"])

class GJKCollisionDetector:
    def __init__(self, min_dist_eps: float = 1e-4, max_num_iters: int = 16, debug: bool = False):
        self.min_dist_eps = min_dist_eps
        self.max_num_iters = max_num_iters
        self.debug = debug
    
    def collide(self, obj_a: PhysicsObject, obj_b: PhysicsObject, simplex: Simplex|None = None) -> CollisionData:
        # 0) Initialization phase, create a simplex if one is not already provided  
        if not simplex: 
            simplex = Simplex() 
            dir = glm.normalize(obj_b.pos - obj_a.pos)
            p, verts_idx = self.__get_minkowski_vert(obj_a, obj_b, dir)
            simplex.add_point(p, verts_idx)

        # 1) Search for collision upto  max number of iters 
        hit = False
        iter = 0
        while iter < self.max_num_iters: 
            print(f"---- Iter {iter}, simplex verts {len(simplex.verts)} --- ")
            # 1.1) compute closest point to CH of simplex, and reduce simplex
            p = simplex.find_closest_point_on_simplex()
            p_d = glm.length2(p)

            # 1.2) if p is the origin, exit (collision case)
            if p_d < self.min_dist_eps**2: 
                hit = True
                break

            # 1.3) get a new support point v in direction of -p
            v, vert_idx = self.__get_minkowski_vert(obj_a, obj_b, -p)
            v_d = glm.dot(v, p)
            print(p_d, v_d)
            if simplex.check_contains(vert_idx) or v_d >= p_d:
                break # going nowhere just exit
            
            # 1.4) add point to simplex 
            simplex.add_point(v, vert_idx)
            iter +=1

            print(simplex.verts_idx, simplex.sub_simplex_indices, simplex.barycentric_coords)
        print(f"------ Exit in {iter+1} iterations -----")

        # 2) (EPA or similar is needed to extract contact normal) and surface points 
        # Use barycentric coordinates in simplex to compute collision mesh points 
        closest_points = self.__get_closest_points(obj_a, obj_b, simplex)
        return CollisionData(hit, simplex, closest_points)

    def __get_minkowski_vert(self, obj_a: PhysicsObject, obj_b: PhysicsObject, dir: glm.vec3) -> tuple[glm.vec3, tuple[int, int]]:
        pa, pa_idx = obj_a.get_support_point(dir)
        pb, pb_idx = obj_b.get_support_point(-dir) 
        return (pa - pb, (pa_idx, pb_idx))
    
    def __get_closest_points(self, obj_a: PhysicsObject, obj_b:PhysicsObject, simplex: Simplex) -> glm.vec3:
        local_cp_a, local_cp_b = glm.vec3(0.0), glm.vec3(0.0)
        for i in range(len(simplex.sub_simplex_indices)):
            vert_idx_a, vert_idx_b = simplex.verts_idx[i]
            w = simplex.barycentric_coords[i]
            
            local_cp_a = local_cp_a + w * obj_a.collision_mesh.verts[vert_idx_a]
            local_cp_b = local_cp_b + w * obj_b.collision_mesh.verts[vert_idx_b]

        global_cp_a = obj_a.point_to_global(local_cp_a)
        global_cp_b = obj_b.point_to_global(local_cp_b)

        return (global_cp_a, global_cp_b) 

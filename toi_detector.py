import math
import glm
from gjk import GJKCollisionDetector, CollisionData
from simplex import Simplex
from physics_object import PhysicsObject

class TOIDetector: 
    def __init__(self, dist_eps:float=1e-3, max_num_timesteps:int=10):
        self.dist_eps = dist_eps # if we reach this distance or below we consider the two objects are touching
        self.max_num_timesteps = max_num_timesteps

        # use default settings of GJK for now  
        self.collision_detector = GJKCollisionDetector()
    
    def detect(self, obj_a: PhysicsObject, obj_b: PhysicsObject, t_max:float=100, reuse_simplex: bool = True) -> tuple[bool, float]:
        # Save object states 
        obj_a.save(), obj_b.save()
        
        # Solution must be found in fixed number of iters 
        prev_simplex_verts_idx = None
        t = 0
        t_steps = [] 
        for i in range(self.max_num_timesteps):
            # store time of current iteration
            t_steps.append(t)

            # 0) reuse simplex from previous iteration to speed things up
            simplex = None
            if prev_simplex_verts_idx and reuse_simplex:
               simplex = self.__construct_simplex_from_indices(obj_a, obj_b, prev_simplex_verts_idx)

            # 1) Compute distance between objects:
            collision_data = self.collision_detector.collide(obj_a, obj_b, simplex)
            cp_a, cp_b = collision_data.closest_points
            prev_simplex_verts_idx = collision_data.simplex.verts_idx

            # 2) Compute separating distance and normal normal from a to b
            n = cp_b - cp_a # not normalized
            n_sq = glm.dot(n, n)

            print(collision_data, n_sq)
            if n_sq < self.dist_eps**2 or collision_data.hit:
                obj_a.restore(), obj_b.restore()
                return (True, t, t_steps)  

            # 3) Conservative advancement 
            #  (pc_a(t')-pc_a).n - (pc_b(t')-pc_b).n = d 
            #  ( vc_a*t' + 0.5 *ac_a*t'^2 - vc_b*t' - 0.5*vc_b*t'^2).n*d = d^2
            #  c = - d^2 
            #  b = (vc_a - vc_b).n*d
            #  a = 0.5 *( ac_a - ac_b).n*d
            a = 0.5 * glm.dot(obj_a.accel - obj_b.accel, n)
            b = glm.dot(obj_a.vel - obj_b.vel, n)
            c = - n_sq 
            dt_sol = _solve_quadratic(a, b, c)
            valid_dt_sol = [dt for dt in dt_sol if dt > 0 and t + dt <= t_max]

            # 4) Check solutions, advance time
            if not valid_dt_sol:
                obj_a.restore(), obj_b.restore()
                return (False, t_max, t_steps) # not hit

            dt = min(valid_dt_sol)
            t = t + dt 
            # 5) Update physics object positions
            obj_a.update(dt), obj_b.update(dt) 
            

        # Ran out of iters (that most likely means we are close to a surface)
        obj_a.restore(), obj_b.restore()
        return (True, t, t_steps)

    def __construct_simplex_from_indices(self, object_a: PhysicsObject, object_b: PhysicsObject, verts_idx: list[tuple[int]])-> Simplex:  
        simplex = Simplex()
        for vert_a_idx, vert_b_idx in verts_idx:
            minkowski_vert = object_a.get_transformed_mesh_vert(vert_a_idx) \
                             - object_b.get_transformed_mesh_vert(vert_b_idx) 
            simplex.add_point(minkowski_vert, (vert_a_idx, vert_b_idx))
        return simplex

def _solve_quadratic(a: float, b: float, c: float) -> tuple[float]|None:
    # degenerate case, attempt to use linear solver
    if a == 0.0: # avoid NaN
        return _solve_linear(b, c)

    # compute discriminant
    disc_sq = b**2 - 4*a*c
    if disc_sq < 0: 
        return () 

    # due to rounding errors disc_sq is rarely pure 0
    disc = math.sqrt(disc_sq)
    x0 = (-b + disc) / (2*a)
    x1 = (-b - disc) / (2*a)
    return (x0, x1)

def _solve_linear(a: float, b:float) -> tuple[float]|None:
    # degenerate case, inf amount of x solutions 
    if a == 0.0: # avoid NaN
       return () 

    # solve a*x + b = 0
    return (-b / a, )
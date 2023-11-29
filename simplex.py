import glm
import sys

class Simplex:
    """
        Create empty simplex 
    """
    def __init__(self):
        # vertices on minkowski difference  
        self.verts = []
        self.num_verts = 0

        # tuples of indexes (corresponding to the support points on each target mesh)
        self.verts_idx = []

        # indices of sub-simplex which contain the closest point (and should not be pruned)
        self._keep_indices = ()

    """
        Store point (from surface of Minkowski difference) and the corresponding indices 
        from collision_mesh_1 and mesh_collision_mesh2
    """
    def add_point(self, vert:glm.vec3, vert_idx:tuple[int]):
        self.verts.append(vert)
        self.verts_idx.append(vert_idx)
        self.num_verts += 1

    """
        Finds closest point on simplex to the reference point ref_point and 
        removes all redundant simplex points.
    """
    def find_closest_point_on_simplex(self, p:glm.vec3 = glm.vec3(0.0)) -> glm.vec3:
        q = None 
        if self.num_verts == 1:
            q = self.verts[0]
            self._keep_indices = (0)
        elif self.num_verts == 2:
            q = self.__find_closest_point_on_segment(p)
        elif self.num_verts == 3:
            q = self.__find_closest_point_on_triangle(p)
        elif self.num_verts == 4:
            q = self.__find_closest_point_on_tetrahedron(p)
        else: 
            raise Exception(f"Invalid number of verts({self.num_verts}) in simplex!!")

        self.__prune_redundant_verts()

        return q


    # Note: Closest point functions where shamelessly yoinked from 
    # Christer Ericson's book "Real-Time Collision Detection"

    def __find_closest_point_on_segment(self, p: glm.vec3) -> glm.vec3:
        # obtain relevant points
        a, b  = self.verts 

        ab = b - a
        # project p onto ab, but deferring divide by dot(ab, ab)
        t = glm.dot(p - a, ab)
        if t <= 0.0:
            # p projects outside the [a,b] interval, on the a side; return a 
            self._keep_indices = (0)
            return a;  
         
        denom = glm.dot(ab, ab); # Always nonnegative since denom = ||ab||∧2
        if t >= denom: 
            # p projects outside the [a,b] interval, on the b side; clamp to b
            self._keep_indices = (1)
            t = 1.0
            return b; 
            
        # p projects inside the [a,b] interval; must do deferred divide now
        self._keep_indices = (0, 1)
        t = t / denom
        return a + t * ab

    def __find_closest_point_on_triangle(self, p: glm.vec3, ai: int = 0, bi: int = 1, ci: int = 2) -> glm.vec3:
        # obtain relevant points 
        a, b, c = self.verts[ai], self.verts[bi], self.verts[ci]
        
        # Check if P in vertex region outside A
        ab = b - a
        ac = c - a
        ap = p - a

        d1 = glm.dot(ab, ap)
        d2 = glm.dot(ac, ap)
        if d1 <= 0.0 and d2 <= 0.0:
            self._keep_indices = (ai)
            return a # barycentric coordinates (1,0,0)

        # Check if P in vertex region outside B
        bp = p - b
        d3 = glm.dot(ab, bp)
        d4 = glm.dot(ac, bp)
        if d3 >= 0.0 and d4 <= d3:
            self._keep_indices = (bi)
            return b # barycentric coordinates (0,1,0)

        # Check if P in edge region of AB, if so return projection of P onto AB
        vc = d1*d4 - d3*d2
        if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0: 
            self._keep_indices = (ai, bi)
            v = d1 / (d1 - d3)
            return a + v * ab # barycentric coordinates (1-v,v,0)
        
        # Check if P in vertex region outside C
        cp = p - c
        d5 = glm.dot(ab, cp)
        d6 = glm.dot(ac, cp)
        if d6 >= 0.0 and d5 <= d6:
            self._keep_indices = (ci)
            return c # barycentric coordinates (0,0,1)
                
        # Check if P in edge region of AC, if so return projection of P onto AC
        vb = d5*d2 - d1*d6
        if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
            self._keep_indices = (ai, ci)
            w = d2 / (d2 - d6)
            return a + w * ac  # barycentric coordinates (1-w,0,w)
        
        # Check if P in edge region of BC, if so return projection of P onto BC
        va = d3*d6 - d5*d4
        if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
            self._keep_indices = (bi, ci)
            w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
            return b + w * (c - b) # barycentric coordinates (0,1-w,w)

        # P inside face region. Compute Q through its barycentric coordinates (u,v,w)
        self._keep_indices = (ai, bi, ci)
        denom = 1.0 / (va + vb + vc)
        v = vb * denom
        w = vc * denom
        return a + ab * v + ac * w # = u*a + v*b + w*c, u = va * denom = 1.0f - v - w

    def __point_outside_of_plane(self, p: glm.vec3, a: glm.vec3, b: glm.vec3, c: glm.vec3, d: glm.vec3):
        signd = glm.dot(d - a, glm.cross(b - a, c - a)); #[AD AB AC]
        signp = glm.dot(p - a, glm.cross(b - a, c - a)); #[AP AB AC]
        # Points on opposite sides if expression signs are opposite
        return signp * signd < 0.0

    def __find_closest_point_on_tetrahedron(self, p: glm.vec3) -> glm.vec3:
        a, b, c, d = self.verts

        # start out assuming point inside all halfspaces, so closest to itself
        closest_pt = p
        best_sq_dist = sys.float_info.max 

        # if point outside face abc then compute closest point on abc
        if self.__point_outside_of_plane(p, a, b, c, d):
            q = self.__find_closest_point_on_triangle(p, a, b, c)
            sq_dist = glm.dot(q - p, q - p)

            # update best closest point if (squared) distance is less than current best
            if sq_dist < best_sq_dist:
                best_sq_dist, closest_pt = sq_dist, q
        
        # Repeat test for face acd
        if self.__point_outside_of_plane(p, a, c, d, b):
            q = self.__find_closest_point_on_triangle(p, a, c, d)
            if sq_dist < best_sq_dist:
                best_sq_dist, closest_pt = sq_dist, q
        
        # Repeat test for face adb
        if self.__point_outside_of_plane(p, a, d, b, c):
            q = self.__find_closest_point_on_triangle(p, a, d, b)
            if sq_dist < best_sq_dist:
                best_sq_dist, closest_pt = sq_dist, q

        # Repeat test for face bdc
        if self.__point_outside_of_plane(p, b, d, c, a):
            q = self.__find_closest_point_on_triangle(p, b, d, c)
            if sq_dist < best_sq_dist:
                best_sq_dist, closest_pt = sq_dist, q

        return closest_pt

    """
        Remove vertices from simplex which are not in good_indices list 
    """    
    def __prune_redundant_verts(self):
        self.num_verts = len(self._keep_indices)
        self.verts = [self.verts[i] for i in self._keep_indices]
        self.verts_idx = [self.verts_idx[i] for i in self._keep_indices] 


def debug_tests():
    simplex = Simplex()
    simplex.add_point(glm.vec3(1), (0, 2))   
    simplex.add_point(glm.vec3(2), (0, 2))   
    simplex.add_point(glm.vec3(3), (0, 2))   
    print(simplex)

    simplex._Simplex__prune_redundant_verts()
    print(simplex.verts)

if __name__ == "__main__":
    debug_tests()
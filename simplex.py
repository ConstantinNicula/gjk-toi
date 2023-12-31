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
        self.sub_simplex_indices = ()

        # barycentric coordinates of points in sub-simplex  
        self.barycentric_coords = ()  

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
            q, self.sub_simplex_indices, self.barycentric_coords = self.verts[0], (0, ), (1.0, )
        elif self.num_verts == 2:
            q, self.sub_simplex_indices, self.barycentric_coords = self.__find_closest_point_on_segment(p)
        elif self.num_verts == 3:
            q, self.sub_simplex_indices, self.barycentric_coords = self.__find_closest_point_on_triangle(p)
        elif self.num_verts == 4:
            q, self.sub_simplex_indices, self.barycentric_coords = self.__find_closest_point_on_tetrahedron(p)
        else: 
            raise Exception(f"Invalid number of verts({self.num_verts}) in simplex!!")

        return q
    
    def check_contains(self, vert_idx:tuple[int])->bool:
        return vert_idx in self.verts_idx

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
            indices = (0,)
            barycentric = (1.0,)
            return a, indices, barycentric;  
         
        denom = glm.dot(ab, ab); # Always nonnegative since denom = ||ab||∧2
        if t >= denom: 
            # p projects outside the [a,b] interval, on the b side; clamp to b
            indices = (1,)
            barycentric = (1.0,)
            t = 1.0
            return b, indices, barycentric; 
            
        # p projects inside the [a,b] interval; must do deferred divide now
        t = t / denom
        indices = (0, 1)
        barycentric = (1.0 - t, t)
        return a + t * ab, indices, barycentric

    def __find_closest_point_on_triangle(self, p: glm.vec3, ai: int = 0, bi: int = 1, ci: int = 2) -> tuple[glm.vec3, tuple[int], tuple[float]]:
        # obtain relevant points 
        a, b, c = self.verts[ai], self.verts[bi], self.verts[ci]

        # compute edge      
        ab = b - a
        ac = c - a
        bc = c - b

        # compute parametric position s for projection P’ of P on AB,
        # P’ = A + s*AB, s = snom/(snom+sdenom)
        snom = glm.dot(p - a, ab)
        sdenom = glm.dot(p - b, a - b)

        #Compute parametric position t for projection P’ of P on AC,
        # P’ = A + t*AC, t = tnom/(tnom+tdenom)
        tnom = glm.dot(p - a, ac)
        tdenom = glm.dot(p - c, a - c)

        # Vertex region A early out
        if snom <= 0.0 and tnom <= 0.0: 
            indices = (ai, )
            barycentric = (1.0,)
            return a, indices, barycentric 

        # compute parametric position u for projection P’ of P on BC,
        # P’ = B + u*BC, u = unom/(unom+udenom)
        unom = glm.dot(p - b, bc)
        udenom = glm.dot(p - c, b - c)
        
        # Vertex region B early out
        if sdenom <= 0.0 and unom <= 0.0:
            indices = (bi, )
            barycentric = (1.0, )
            return b, indices, barycentric
        
        # Vertex region C early out
        if tdenom <= 0.0 and udenom <= 0.0:
            indices = (ci, )
            barycentric = (1.0, )
            return c, indices, barycentric# Vertex region early out

        # Compute plane normal 
        n = glm.cross(b - a, c - a)

        # P is outside (or on) AB if the triple scalar product [N PA PB] <= 0
        vc = glm.dot(n, glm.cross(a - p, b - p))

        # If P outside AB and within feature region of AB,
        # return projection of P onto AB
        if vc <= 0.0 and snom >= 0.0 and sdenom >= 0.0:
            s = snom / (snom + sdenom)
            indices = (ai, bi)
            barycentric = (1 - s, s)
            return a +  s * ab, indices, barycentric

        # P is outside (or on) BC if the triple scalar product [N PB PC] <= 0
        va = glm.dot(n, glm.cross(b - p, c - p))

        # If P outside BC and within feature region of BC,
        # return projection of P onto BC
        if va <= 0.0 and unom >= 0.0 and udenom >= 0.0:
            u = unom / (unom + udenom)
            indices = (bi, ci)
            barycentric = (1 - u, u)           
            return b +  u * bc, indices, barycentric

        # P is outside (or on) CA if the triple scalar product [N PC PA] <= 0
        vb = glm.dot(n, glm.cross(c - p, a - p))

        # If P outside CA and within feature region of CA,
        # return projection of P onto CA
        if vb <= 0.0 and tnom >= 0.0 and tdenom >= 0.0:
            t = tnom / (tnom + tdenom)
            indices = (ai, ci)
            barycentric = (1 - t, t)
            return a +  t * ac, indices, barycentric

        # P must project inside face region. Compute Q using barycentric coordinates
        u = va / (va + vb + vc)
        v = vb / (va + vb + vc)
        w = 1.0 - u - v; # = vc / (va + vb + vc)
        indices = (ai, bi, ci)
        barycentric = (u, v, w)
        return u * a + v * b + w * c, indices, barycentric

    
    def __point_outside_of_plane(self, p: glm.vec3, a: glm.vec3, b: glm.vec3, c: glm.vec3, d: glm.vec3):
        signd = glm.dot(d - a, glm.cross(b - a, c - a)); #[AD AB AC]
        signp = glm.dot(p - a, glm.cross(b - a, c - a)); #[AP AB AC]
        # Points on opposite sides if expression signs are opposite
        return signp * signd <= 0.0

    def __find_closest_point_on_tetrahedron(self, p: glm.vec3) -> tuple[glm.vec3, tuple[int], tuple[float]]:
        ai, bi, ci, di = range(4) 
        a, b, c, d = self.verts

        # start out assuming point inside all halfspaces, so closest to itself
        closest_pt = p
        best_sq_dist = sys.float_info.max 
        ret_indices, ret_barycentric = (0, 1, 2, 3), (0.25, 0.25, 0.25, 0.25)
        
        # if point outside face abc then compute closest point on abc
        if self.__point_outside_of_plane(p, a, b, c, d):
            q, indices, barycentric = self.__find_closest_point_on_triangle(p, ai, bi, ci)
            sq_dist = glm.dot(q - p, q - p)
            if sq_dist < best_sq_dist:
                best_sq_dist, closest_pt = sq_dist, q
                ret_indices, ret_barycentric = indices, barycentric
        
        # Repeat test for face acd
        if self.__point_outside_of_plane(p, a, c, d, b):
            q, indices, barycentric = self.__find_closest_point_on_triangle(p, ai, ci, di)
            sq_dist = glm.dot(q - p, q - p)
            if sq_dist < best_sq_dist:
                best_sq_dist, closest_pt = sq_dist, q
                ret_indices, ret_barycentric = indices, barycentric
        
        # Repeat test for face adb
        if self.__point_outside_of_plane(p, a, d, b, c):
            q, indices, barycentric = self.__find_closest_point_on_triangle(p, ai, di, bi)
            sq_dist = glm.dot(q - p, q - p)
            if sq_dist < best_sq_dist:
                best_sq_dist, closest_pt = sq_dist, q
                ret_indices, ret_barycentric = indices, barycentric
        
        # Repeat test for face bdc
        if self.__point_outside_of_plane(p, b, d, c, a):
            q, indices, barycentric = self.__find_closest_point_on_triangle(p, bi, di, ci)
            sq_dist = glm.dot(q - p, q - p)
            if sq_dist < best_sq_dist:
                best_sq_dist, closest_pt = sq_dist, q
                ret_indices, ret_barycentric = indices, barycentric

        return closest_pt, ret_indices, ret_barycentric

    """
        Remove vertices from simplex which are not in good_indices list
        Note: result is only valid if find_closest_point_on_simplex was called previously
    """    
    def reduce(self):
        self.num_verts = len(self.sub_simplex_indices)
        self.verts = [self.verts[i] for i in self.sub_simplex_indices]
        self.verts_idx = [self.verts_idx[i] for i in self.sub_simplex_indices] 


def debug_tests():
    simplex = Simplex()
    simplex.add_point(glm.vec3(0, 0, 0), (0, 0))   
    simplex.add_point(glm.vec3(1, 0, 0), (0, 0))   
    simplex.add_point(glm.vec3(1, 1, 0), (0, 0))   
    print(simplex)

    q = simplex.find_closest_point_on_simplex(glm.vec3(2, 3, 0))
    print(q)

if __name__ == "__main__":
    debug_tests()
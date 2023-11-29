import numpy as np 

class MinkowskiMesh2D:
    def __init__(self, verts_1, verts_2):
        self.mesh = self.__brute_force_compute(verts_1, verts_2) 

    ''' Compute A - B Minkowski difference/sum:
            - take all points a_i in A and subtract all points b_j in B
            - take convex hull of result
    '''
    def __brute_force_compute(self, verts1, verts2):
        #convert to np.array 
        verts_1 = np.asarray(verts1)
        verts_2 = np.asarray(verts2)

        minkowski_diff = np.repeat(verts1, verts2.shape[0], 0) - np.tile(verts2, (verts1.shape[0], 1))
        return ConvexHull.compute_from_points(minkowski_diff)
         

class ConvexHull: 
    @staticmethod
    def compute_from_points(verts):
        # convert to np.array 
        verts = np.asarray(verts)
        
        # make sure array has shape (N, 2)
        assert len(verts.shape) == 2 and verts.shape[1] == 2 

        # starting point farthest point left towards -x 
        Hi = [np.argmin(verts[:, 0])]
        while True:
            h_st = Hi[-1]; # last point on hull is start of new seg
            h_end = (h_st + 1) % len(verts) 
            for h_c in range(0, len(verts)):
                if h_c == h_st or h_c == h_end:
                    continue
                # check if h_c is to the left hand side of segment[h_st, h_end]
                if compute_cross_2d(verts[h_st], verts[h_end], verts[h_c]) < 0: 
                    h_end = h_c
            if h_end == Hi[0]:
                break;               
            Hi.append(h_end)
        return verts[Hi]


def compute_cross_2d(p_st, p_end, p_c): 
    return (p_end[0] - p_st[0])*(p_c[1] - p_st[1])  \
            - (p_end[1] - p_st[1])*(p_c[0] - p_st[0])              


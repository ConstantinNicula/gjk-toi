import glm
import glm_utils

class CollisionMesh: 
    """
        Create a collision mesh from a list of vertices and optional model (R, t)
    """
    def __init__(self, verts: list[glm.vec3]):
        self.verts = verts  
   
    def get_transformed_verts(self, R: glm.mat3, t: glm.vec3) -> list[glm.vec3]:
        return list(map(lambda v: R * v + t, self.verts))

    def get_support_point(self, support_dir:glm.vec3):
        support_dir = glm.normalize(support_dir)

        best_idx = 0 
        max_dist = glm.dot(support_dir, self.verts[0])
        
        #this is dumb and slow in python 
        for v, idx in enumerate(self.verts):
            dist = glm.dot(v, support_dir)
            if dist > max_dist:
                max_dist = dist 
                best_idx = idx

        return (self.verts[best_idx], best_idx) 
            


def test():
    mesh_verts = glm_utils.to_glm_vec_list([[1, 1], [2, 1], [2, 2], [1, 2], [0.5, 1.5]])

    mesh = CollisionMesh(mesh_verts)
    world_verts = mesh.get_transformed_verts( glm_utils.rotation_from_euler(0, 0, 90), glm.vec3(0, 0, 1))
    print(world_verts) 


if __name__ == "__main__":
    test()
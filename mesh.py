import glm


class Mesh: 
    """
        Create a mesh from a list of vertices and an optional model transform (R, t)
    """
    def __init__(self, verts: list[glm.vec3], R:glm.mat3 = glm.mat3(1.0), t:glm.vec3 = glm.vec3(0.0)):
        self.verts = verts  
        self.R = R 
        self.t = t

    def set_rotation(self, R:glm.mat3):
        self.R = R

    def set_rotation_euler(self, degx: float, degy: float, degz: float):
        Rx = glm.rotate(glm.radians(degx), glm.vec3(1.0, 0.0, 0.0))
        Ry = glm.rotate(glm.radians(degy), glm.vec3(0.0, 1.0, 0.0))
        Rz = glm.rotate(glm.radians(degz), glm.vec3(0.0, 0.0, 1.0))
        self.R = glm.mat3(Rx * Ry * Rz)

    def set_translation(self, t:glm.vec3):
        self.t = t
    
    def get_world_verts(self) -> list[glm.vec3]:
        return list(map(lambda v: self.R * v + self.t, self.verts))

class MeshUtils:
    @staticmethod
    def to_glm_vec_list(verts: any) -> list[glm.vec3]:
        return map(MeshUtils.to_glm_vec, verts)

    @staticmethod
    def to_glm_vec(v: any) -> glm.vec3:
        assert len(v) == 2 or len(v) == 3
        if len(v) == 2:
            return glm.vec3(v[0], v[1], 0)
        else: 
            return glm.vec3(v[0], v[1], v[2])
        
def test():
    mesh_verts = MeshUtils.to_glm_vec_list([[1, 1], [2, 1], [2, 2], [1, 2], [0.5, 1.5]])
    mesh = Mesh(mesh_verts)
    mesh.set_rotation_euler(0, 0, 90)
    mesh.set_translation(glm.vec3(0, 0, 1))
    print(mesh.get_world_verts())

if __name__ == "__main__":
    test()
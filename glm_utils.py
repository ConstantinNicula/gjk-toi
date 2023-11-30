import glm

def to_glm_vec_list(verts: any) -> list[glm.vec3]:
    return list(map(to_glm_vec, verts))

def to_glm_vec(v: any) -> glm.vec3:
    assert len(v) == 2 or len(v) == 3
    if len(v) == 2:
        return glm.vec3(v[0], v[1], 0)
    else: 
        return glm.vec3(v[0], v[1], v[2])

def to_2d_point(v: glm.vec3) -> list[float]:
    return [v.x, v.y]

def to_2d_point_list(verts: list[glm.vec3]) ->list[list[float, float]]:
    return list(map(to_2d_point, verts))

def to_3d_point(v: glm.vec3) -> list[float]:
    return [v.x, v.y, v.z]

def to_3d_point_list(verts: list[glm.vec3]) ->list[list[float, float, float]]:
    return list(map(to_3d_point, verts))


def rotation_from_euler(degx: float, degy: float, degz: float) -> glm.mat3:
    Rx = glm.rotate(glm.radians(degx), glm.vec3(1.0, 0.0, 0.0))
    Ry = glm.rotate(glm.radians(degy), glm.vec3(0.0, 1.0, 0.0))
    Rz = glm.rotate(glm.radians(degz), glm.vec3(0.0, 0.0, 1.0))
    return glm.mat3(Rx * Ry * Rz)


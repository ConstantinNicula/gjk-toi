import glm
import glm_utils

from collision_mesh import CollisionMesh 

class PhysicsObject:
    def __init__(self, collision_mesh: CollisionMesh, 
                    scale: glm.vec3 = glm.vec3(1.0), rot: glm.mat3 = glm.mat3(1.0), 
                    pos: glm.vec3 = glm.vec3(0.0), vel: glm.vec3 = glm.vec3(0.0), 
                    accel: glm.vec3 = glm.vec3(0.0)):

        self.collision_mesh = collision_mesh

        # static properties 
        self.scale = scale
        self.rot = rot

        # dynamic properties 
        self.pos = pos
        self.vel = vel
        self.accel = accel

    # update internal state
    def set_rotation(self, rot:glm.mat3):
        self.rot = rot 

    def set_rotation_euler(self, degx: float, degy: float, degz: float):
        self.rot = glm_utils.rotation_from_euler(degx, degy, degz)

    def set_scale(self, scale: glm.vec3): 
        self.scale = scale 

    def set_position(self, pos: glm.vec3):
        self.pos = pos 

    def set_velocity(self, vel: glm.vec3):
        self.vel = vel

    def set_accel(self, accel:glm.vec3):
        self.accel = accel
 
    def update(self, dt: float):
        self.pos = self.pos + self.vel * dt + 0.5 * self.accel * dt**2 
        self.vel = self.vel + self.accel * dt

    def get_support_point(self, global_dir: glm.vec3) -> tuple[glm.vec3, int]:
        # convert direction to local rf  
        local_dir = self.dir_to_local(global_dir)
        # find support point in local rf
        p, idx = self.collision_mesh.get_support_point(local_dir)
        # get point in global rf
        return (self.point_to_global(p), idx)

    def get_glm_transform(self) -> glm.mat4:
        return glm.translate(self.pos) * glm.mat4(self.rot) * glm.scale(self.scale)     
    
    # utility functions for transformations 
    def dir_to_global(self, local_dir: glm.vec3) -> glm.vec3:
        # (M^-1)^T = ((R*S)^-1)^T = (S^-1 * R^T) ^T = R * S^-1 
        # element-wise division (local_dir / self.scale) = dx/sx, dy/sy, dz/sz
        return self.rot * (local_dir / self.scale)

    def dir_to_local(self, global_dir: glm.vec3) -> glm.vec3:
        # required matrix is S * R^T
        return (glm.transpose(self.rot) * global_dir) * self.scale; 

    def point_to_global(self, local_point: glm.vec3) -> glm.vec3:
        # pg = R * S * lp + t 
        return self.rot * (self.scale * local_point) + self.pos

    def point_to_local(self, global_point: glm.vec3) -> glm.vec3:
        # pl = S^-1*R^T * (pg - t)
        return (glm.transpose(self.rot) * (global_point - self.pos)) / self.scale 

    # debugging 
    def get_transformed_mesh_vert(self, vert_idx: int) -> glm.vec3:
        return self.point_to_global(self.collision_mesh[vert_idx]) 

    def get_transformed_mesh(self) -> list[glm.vec3]:
        return self.collision_mesh.get_transformed_verts(self.rot, self.scale, self.pos) 
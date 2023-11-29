import glm
import glm_utils

from collision_mesh import CollisionMesh 

class PhysicsObject:
    def __init__(self, collision_mesh: CollisionMesh, rot: glm.mat3, pos: glm.vec3, 
                    vel: glm.vec3, accel: glm.vec3):
        self.collision_mesh = collision_mesh

        self.rot = rot
        self.pos = pos

        self.vel = vel
        self.accel = accel

    def set_rotation(self, rot:glm.mat3):
        self.rot = rot 

    def set_rotation_euler(self, degx: float, degy: float, degz: float):
        self.rot = glm_utils.rotation_from_euler(degx, degy, degz)

    def set_position(self, pos:glm.vec3):
        self.pos = pos 
 
    def update(self, dt: float):
        self.pos = self.pos + self.vel * dt + self.accel * dt**2 
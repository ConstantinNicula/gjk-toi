import typing
import pickle, os
import glm, trimesh
import math, time
from collections import namedtuple

# own libs
import glm_utils
from physics_object import PhysicsObject
from collision_mesh import CollisionMesh
from gjk import GJKCollisionDetector, CollisionData
from simplex import Simplex

# display stuff
import pyqtgraph as pg 
from pyqtgraph.dockarea import *
import pyqtgraph.opengl as gl 
from PyQt5 import QtWidgets, QtGui, QtCore

class SuperSpinner(QtWidgets.QDoubleSpinBox):
    def __init__(self):
        super(SuperSpinner, self).__init__()

        self.mouseStartPosY = 0
        self.startValue = 0

    def mousePressEvent(self, e):
        super(SuperSpinner, self).mousePressEvent(e)
        self.mouseStartPosY = e.pos().y()
        self.startValue = self.value()

    def mouseMoveEvent(self, e):
        self.setCursor(QtCore.Qt.SizeVerCursor)

        delta = self.mouseStartPosY - e.pos().y()
        valueOffsetExp =  math.pow(1.015, abs(delta)) -1.0
        valueOffset = math.copysign(1, delta) * valueOffsetExp
        self.setValue(self.startValue + valueOffset)

    def mouseReleaseEvent(self, e):
        super(SuperSpinner, self).mouseReleaseEvent(e)
        self.unsetCursor()


ObjectState = namedtuple("ObjectState", ["rotation", "scale", "position", "velocity", "accel"])

class DebugVisualizer3D: 
    def __init__(self, name):
        name = name if name else "Default name" 

        pg.setConfigOptions(antialias=True, useOpenGL=True)
        self.app = QtWidgets.QApplication([])

        # toplevel widget 
        self.area = DockArea()
        
        #create window
        self.win = QtWidgets.QMainWindow()
        self.win.setCentralWidget(self.area)
        self.win.setWindowTitle(name)
        self.win.resize(1600, 700)

        # create dock for 3d viewport 
        self.scene = SceneController("3D View", self.area, location='left') 
        
        # create a settings controllers for both objects
        callback_a = lambda object_state: self.object_state_change(0, object_state)
        self.obj_a_settings = ObjectSettingsController("Object A Settings", self.area, callback_a, location='right')
        callback_b = lambda object_state: self.object_state_change(1, object_state)
        self.obj_b_settings = ObjectSettingsController("Object B Settings", self.area, callback_b, location='bottom', ref_dock=self.obj_a_settings.dock)

        # TO DO: add other settings controllers 
    


        # storage for internal data  
        self.physics_objects: list[PhysicsObject] = []
        self.__add_physics_object(self.obj_a_settings.state_values, 'cube')
        self.__add_physics_object(self.obj_b_settings.state_values, 'cylinder')

        self.collision_detector = GJKCollisionDetector()
        self.prev_simplex_idx = None

        # display window  
        self.win.show()
        self.last_time = time.time()

        # set initial conditions 
        # self.obj_a_settings.set_state(ObjectState(
        #     (0.00, 45.41, 0.00),
        #     (1.28, 1.35, 1.00),
        #     (9.00, 0.00, 0.00),
        #     (5.00, 5.00, 4.00), 
        #     (0.00, 0.00, -9.81)
        # ))

        # self.obj_b_settings.set_state(ObjectState(
        #     (-52.31, -13.57, 37.68),
        #     (1.86, 0.61, 1.77),
        #     (10.33, -0.91, 1.13),
        #     (0.00, 0.00, 0.62), 
        #     (0.00, 0.00, 0.00), 
        # ))

        self.__load_settings()

    def __add_physics_object(self, state: ObjectState, object_type:str):
        # check options 
        display_mesh_item, mesh_vertices = None, None
        if object_type == 'cylinder': 
            display_mesh_item, mesh_vertices = self.scene.create_cylinder_object() 
        elif object_type == 'cube':
            display_mesh_item, mesh_vertices = self.scene.create_cube_object() 
        else:
            raise Exception(f"Unknown object_type {object_type}")

        collision_mesh = CollisionMesh(glm_utils.to_glm_vec_list(mesh_vertices))
        physics_object = PhysicsObject(collision_mesh)

        self.__update_object_state(display_mesh_item, physics_object, state)
        self.physics_objects.append(physics_object)

    def __update_object_state(self, display_mesh_item: gl.GLMeshItem, physics_object: PhysicsObject, state:ObjectState):
        physics_object.set_rotation(glm_utils.rotation_from_euler(*state.rotation)) 
        physics_object.set_scale(glm_utils.to_glm_vec(state.scale)) 
        physics_object.set_position(glm_utils.to_glm_vec(state.position)) 
        physics_object.set_velocity(glm_utils.to_glm_vec(state.velocity))
        physics_object.set_accel(glm_utils.to_glm_vec(state.accel))

        transform = glm.transpose(physics_object.get_glm_transform()) 
        display_mesh_item.setTransform(transform.to_tuple())

    def __save_settings(self):
        with open('./save.bin', 'wb') as file:
            pickle.dump((self.obj_a_settings.state_values, self.obj_b_settings.state_values), file) 
    
    def __load_settings(self):
        if not os.path.isfile('./save.bin'):
            print('No save file found! Exiting..')
            return

        with open('./save.bin', 'rb') as file:
            config_a, config_b = pickle.load(file)
            self.obj_a_settings.set_state(config_a)
            self.obj_b_settings.set_state(config_b)

    def object_state_change(self, object_id:int, object_state:ObjectState):
        print (object_id, object_state)

        # remove previous data
        self.scene.clear_debug_data()

        # perform position update  
        mesh_item = self.scene.object_meshes[object_id]
        physics_object = self.physics_objects[object_id]
        self.__update_object_state(mesh_item, physics_object, object_state)

        # lets try to run gjk 
        starting_simplex = None 
        if self.prev_simplex_idx:
            starting_simplex = self.__construct_simplex_from_indices(self.physics_objects[0], self.physics_objects[1], self.prev_simplex_idx) 
        ret = self.collision_detector.collide(self.physics_objects[0], self.physics_objects[1], starting_simplex)
        self.prev_simplex_idx = ret.simplex.verts_idx
        print(ret)

        # display closest points
        if not ret.hit: 
            pts = glm_utils.to_3d_point_list(ret.closest_points)
            self.scene.create_debug_line(pts, (1.0, 0.0, 0.0, 1.0), 4)     
        
        # display trajectories 
        self.__display_trajectory(self.physics_objects[0], (1.0, 0.0, 0.0, 1.0))
        self.__display_trajectory(self.physics_objects[1], (0.0, 1.0, 0.0, 1.0))

        curr_time = time.time()
        print (f"elapsed {curr_time - self.last_time}s")
        self.last_time = curr_time

    def __display_trajectory(self, physics_object: PhysicsObject, color:tuple[float]):
        pts = glm_utils.to_3d_point_list(physics_object.compute_trajectory(10, 100))
        self.scene.create_debug_line(pts, color, 8)

    def __construct_simplex_from_indices(self, object_a: PhysicsObject, object_b: PhysicsObject, verts_idx: list[tuple[int]])-> Simplex:  
        simplex = Simplex()
        for vert_a_idx, vert_b_idx in verts_idx:
            minkowski_vert = object_a.get_transformed_mesh_vert(vert_a_idx) \
                             - object_b.get_transformed_mesh_vert(vert_b_idx) 
            simplex.add_point(minkowski_vert, (vert_a_idx, vert_b_idx))
        return simplex

    def display(self):
        self.app.exec_()
        self.__save_settings()



class SceneController:
    def __init__(self, name: str, parent_area: DockArea, size:tuple[int, int] = (10, 2), location:str = 'left'):
        # create a dock and pin it to the parent
        self.dock = Dock(name, parent_area, size)
        parent_area.addDock(self.dock, location)
        
        # create view widget and pin it to own dock
        self.view_widget = gl.GLViewWidget()
        self.dock.addWidget(self.view_widget)

        # create xy plane grid and add it to view
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.view_widget.addItem(grid)

        # storage for render objects 
        self.object_meshes = []
        self.debug_meshes = []
        self.debug_lines = []

    def __create_trimesh(self, mesh: trimesh.Trimesh) -> gl.GLMeshItem:
        mesh_data = gl.MeshData(mesh.vertices, mesh.faces[:, ::-1])
        mesh_item = gl.GLMeshItem(meshdata = mesh_data, drawFaces = True, smooth = False, computeNormals = True, shader='shaded')
        self.view_widget.addItem(mesh_item)
        return mesh_item 

    def create_cylinder_object(self) -> gl.GLMeshItem:
        cylinder_mesh = trimesh.creation.cylinder(1, 1, 20)
        mesh_item = self.__create_trimesh(cylinder_mesh)
        self.object_meshes.append(mesh_item)
        return mesh_item, cylinder_mesh.vertices.tolist()

    def create_cube_object(self) -> gl.GLMeshItem:
        box_mesh = trimesh.creation.box(extents=[1.0, 1.0, 1.0])
        mesh_item = self.__create_trimesh(box_mesh)
        self.object_meshes.append(mesh_item)
        return mesh_item, box_mesh.vertices.tolist()
    
    def create_debug_line(self, points: list[glm.vec3], color:tuple[float], width:float = 2.0) -> gl.GLLinePlotItem: 
        line_item = gl.GLLinePlotItem(pos=points, color=color, width=width, antialias=True, mode='line_strip')
        self.view_widget.addItem(line_item)
        self.debug_lines.append(line_item)
        return line_item

    def clear_debug_data(self):
        for line_item in self.debug_lines:
            self.view_widget.removeItem(line_item)
        for mesh_item in self.debug_meshes:
            self.view_widget.removeItem(mesh_item)
        self.debug_lines, self.debug_meshes = [], []


class ObjectSettingsController:
    # define ranges 
    min_rot, max_rot = (-180, -180, -180), (180, 180, 180)
    min_scale, max_scale = (0, 0, 0), (10, 10, 10)
    min_vals, max_vals = (-1000, -1000, -1000), (1000, 1000, 100)
        
    def __init__(self, name: str, parent_area: DockArea, update_callback,  size:tuple[int, int] = (1,1), location:str = 'right', ref_dock:Dock|None = None):
        # create a dock and pin it to the parent 
        self.dock = Dock(name, parent_area, size)
        if not ref_dock: 
            parent_area.addDock(self.dock, location) 
        else:
            parent_area.addDock(self.dock, location, ref_dock)

        # create a dock layout 
        self.layout = pg.LayoutWidget()
        self.dock.addWidget(self.layout)

        # state storage 
        self.state_values = ObjectState((0.0, 0.0,0.0), (1.0, 1.0, 1.0), (0.0, 0.0,0.0), (0.0, 0.0,0.0), (0.0, 0.0,0.0)) 

        # create controllers 
        self.rotation_controller = _add_labeled_vec3_field(self.layout, "Rotation:",self.min_rot, self.max_rot,self.state_values.rotation, self.__read_all_data)
        self.scale_controller =_add_labeled_vec3_field(self.layout, "Scale:", self.min_scale, self.max_scale, self.state_values.scale, self.__read_all_data)
        
        self.position_controller =_add_labeled_vec3_field(self.layout, "Initial position:", self.min_vals, self.max_vals, self.state_values.position, self.__read_all_data)
        self.velocity_controller = _add_labeled_vec3_field(self.layout, "Initial velocity:", self.min_vals, self.max_vals, self.state_values.velocity, self.__read_all_data)
        self.acceleration_controller = _add_labeled_vec3_field(self.layout, "Acceleration:", self.min_vals, self.max_vals, self.state_values.velocity, self.__read_all_data)
        # notify parent that state has changed 
        self.update_callback = update_callback 

    def __read_vec_data(self, vec_fields: tuple[SuperSpinner]): 
        read_spinner = lambda spinner: float(spinner.value())
        return tuple(map(read_spinner, vec_fields))
    
    def __read_all_data(self, unused):
        self.state_values = ObjectState(
            self.__read_vec_data(self.rotation_controller),
            self.__read_vec_data(self.scale_controller), 
            self.__read_vec_data(self.position_controller), 
            self.__read_vec_data(self.velocity_controller),
            self.__read_vec_data(self.acceleration_controller) 
        )
        self.update_callback(self.state_values) 

    def __update_controller(self, target_state: tuple[float], controller: tuple[SuperSpinner]):
        for field_controller, target_state in zip(controller, target_state):
            field_controller.setValue(target_state)

    def set_state(self, state: ObjectState): 
        self.state_values = state
        self.__update_controller(state.rotation, self.rotation_controller)
        self.__update_controller(state.scale, self.scale_controller)
        self.__update_controller(state.position, self.position_controller)
        self.__update_controller(state.velocity, self.velocity_controller)
        self.__update_controller(state.accel, self.acceleration_controller)


def _create_float_field(range: tuple[float, float], default_value:float, callback) -> SuperSpinner:
    super_spin = SuperSpinner()
    super_spin.setMinimum(range[0])
    super_spin.setMaximum(range[1])
    super_spin.setLocale(QtCore.QLocale("en_US"))
    super_spin.setSingleStep(0.01)
    super_spin.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
    super_spin.setValue(default_value)
    super_spin.valueChanged.connect(callback)
    return super_spin

def _add_labeled_float_field(layout: pg.LayoutWidget, prefix_str:str, 
                                range: tuple[float, float], default_value:float, 
                                callback):
    # create widgets
    label = QtWidgets.QLabel(prefix_str)
    tbox = _create_float_field(range, default_value, callback)

    layout.addWidget(label)
    layout.nextColumn()
    layout.addWidget(tbox)
    layout.nextColumn()

    return tbox

def _add_labeled_vec3_field(parent_layout: pg.LayoutWidget, 
                                label: str,
                                min_range: tuple[float, float, float],
                                max_range: tuple[float, float, float],
                                default_values: tuple[float, float, float],
                                callback):
    # create labels  
    label = QtWidgets.QLabel(label)
    input_layout = pg.LayoutWidget()
    tboxes = []
    for prefix, minv, maxv, defv in zip(("X: ", "Y: ", "Z: "), min_range, max_range, default_values):
        tboxes.append(_add_labeled_float_field(input_layout, prefix, (minv, maxv), defv, callback))

    # create sub-layout for input fields and add input fields to it
    parent_layout.addWidget(label)
    parent_layout.nextRow()
    parent_layout.addWidget(input_layout)
    parent_layout.nextRow()

    return tuple(tboxes)



def main():
    debug_visualizer = DebugVisualizer3D("Test")
    debug_visualizer.display()

if __name__ == "__main__":
    main()
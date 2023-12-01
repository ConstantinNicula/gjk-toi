import glm, glm_utils
import math, time
import trimesh
from collections import namedtuple

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


ObjectState = namedtuple("ObjectState", ["rotation", "scale", "position", "velocity"])

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
        self.win.resize(1200, 700)

        # create dock for 3d viewport 
        self.scene = SceneController("3D View", self.area, location='left') 
        
        # create a settings controllers for both objects
        callback_a = lambda object_state: self.object_state_change(0, object_state)
        self.obj_a_settings = ObjectSettingsController("Object A Settings", self.area, callback_a, location='right')
        callback_b = lambda object_state: self.object_state_change(1, object_state)
        self.obj_b_settings = ObjectSettingsController("Object B Settings", self.area, callback_b, location='bottom', ref_dock=self.obj_a_settings.dock)

        # temp
        self.object_a_mesh = self.scene.create_cylinder_object() 

        # display window  
        self.win.show()
        self.last_time = time.time()

    def __create_physics_object(self, ):
        pass 

    def object_state_change(self, object_id:int, object_state:ObjectState):
        print (object_id, object_state)
        # to do update physics object 
        # update meshes
        translation = glm_utils.to_glm_vec(object_state.position)
        transform = glm.transpose(glm.translate(translation)) 
        self.object_a_mesh.setTransform(transform.to_tuple())
        curr_time = time.time()
        print (f"elapsed {curr_time - self.last_time}s")
        self.last_time = curr_time
    
    def create_3d_view_dock(self):
        pass 

    def add_numerical_input():
        pass 
    def display(self):
        self.app.exec_()

class SceneController:
    def __init__(self, name: str, parent_area: DockArea, size:tuple[int, int] = (3, 3), location:str = 'left'):
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
        self.meshes = []
        self.lines = []

    def add_trimesh(self, mesh: trimesh.Trimesh) -> gl.GLMeshItem:
        mesh_data = gl.MeshData(mesh.vertices, mesh.faces[:, ::-1])
        mesh_item = gl.GLMeshItem(meshdata = mesh_data, drawFaces = True, smooth = False, computeNormals = True, shader='shaded')
        self.view_widget.addItem(mesh_item)
        self.meshes.append(mesh_item)
        return mesh_item 

    def create_cylinder_object(self) -> gl.GLMeshItem:
        cylinder_mesh = trimesh.creation.cylinder(1, 1, 20)
        return self.add_trimesh(cylinder_mesh)

    def create_cube_object(self) -> gl.GLMeshItem:
        box_mesh = trimesh.creation.box(extents=[1.0, 1.0, 1.0])
        return self.add_trimesh(box_mesh)
    
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
        self.state_values = ObjectState((0.0, 0.0,0.0), (1.0, 1.0, 1.0), (9.0, 0.0,0.0), (0.0, 0.0,0.0)) 

        # create controllers 
        self.rotation_controller = _add_labeled_vec3_field(self.layout, "Rotation:",self.min_rot, self.max_rot,self.state_values.rotation, self.__read_all_data)
        self.scale_controller =_add_labeled_vec3_field(self.layout, "Scale:", self.min_scale, self.max_scale, self.state_values.scale, self.__read_all_data)
        self.position_controller =_add_labeled_vec3_field(self.layout, "Initial position:", self.min_vals, self.max_vals, self.state_values.position, self.__read_all_data)
        self.velocity_controller = _add_labeled_vec3_field(self.layout, "Initial velocity:", self.min_vals, self.max_vals, self.state_values.velocity, self.__read_all_data)

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
            self.__read_vec_data(self.velocity_controller) 
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


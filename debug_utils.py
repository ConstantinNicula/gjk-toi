import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.patches import Polygon 

import pyqtgraph as pg 
from pyqtgraph.dockarea import *
import pyqtgraph.opengl as gl 
from PyQt5 import QtWidgets, QtGui, QtCore

class DebugPlotter: 
    def __init__(self, x_bounds=None, y_bounds=None):
        self.fig, self.ax = plt.subplots()
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.polygons = []
        self.polygon_colors = []

    def add_poly(self, verts, color): 
        self.polygons.append(verts)  
        # todo chose random color  
        self.polygon_colors.append(color)

    def display(self): 
        if not self.x_bounds:
            self.x_bounds = self.__compute_limits(0)
        self.ax.set_xlim(self.x_bounds)

        if not self.y_bounds:
            self.y_bounds = self.__compute_limits(1)
        self.ax.set_ylim(self.y_bounds)

        plt.grid()
        
        for poly_verts, poly_color in zip(self.polygons, self.polygon_colors):
            if len(poly_verts) > 2:
                p = Polygon(poly_verts, facecolor=poly_color, alpha=0.2)
                self.ax.add_patch(p)

            #draw points 
            xs, ys = zip(*poly_verts)
            plt.scatter(xs, ys)

            # draw contour
            xs, ys = xs+(xs[0],), ys+(ys[0],)
            plt.plot(xs, ys)
        plt.axis('equal')
        plt.show()

    def __compute_limits(self, axis=0, padding=1):
        # stack all points and find min max 
        all_pts = np.vstack(self.polygons)
        min, max = np.min(all_pts[:, axis]), np.max(all_pts[:, axis])
        min = np.floor(min) - padding
        max = np.ceil(max) + padding
        print(min, max) 
        return min, max 


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
        self.scene = SceneDock("3D View", self.area, location='left') 
        
        # create a settings dock (should be separate class)
        self.obj_a_settings = SettingsDock("Object A Settings", self.area, location='right')
        #self.obj_b_settings = SettingsDock("Object B Settings", self.area, location='bottom', ref_dock=self.obj_a_settings.dock)
        
        # display window  
        self.win.show()

    def create_3d_view_dock(self):
        pass 

    def add_numerical_input():
        pass 
    def display(self):
        self.app.exec_()

class SceneDock:
    def __init__(self, name: str, parent_area: DockArea, size:tuple[int, int] = (3, 1), location:str = 'left', ref_dock:Dock|None = None):
        # create a dock and pin it to the parent
        self.dock = Dock(name, parent_area, size)
        if not ref_dock: 
            parent_area.addDock(self.dock, location)
        else:
            parent_area.addDock(self.dock, location, ref_dock)

        # create view widget and pin it to own dock
        self.view_widget = gl.GLViewWidget()
        self.dock.addWidget(self.view_widget)

        # create xy plane grid and add it to view
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.view_widget.addItem(grid)

        # holder for additional widgets
        self.meshes = []
        self.lines = []

class SettingsDock:
    def __init__(self, name: str, parent_area: DockArea, size:tuple[int, int] = (1,1), location:str = 'right'):
        # create a dock and pin it to the parent 
        self.dock = Dock(name, parent_area, size)
        parent_area.addDock(self.dock, location) 

        # create a dock layout 
        self.layout = pg.LayoutWidget()
        self.dock.addWidget(self.layout)

        _add_labeled_vec3_field(self.layout, "Rotation:", (-10, -10, -10), (10, 10, 10), (0, 0, 0), lambda x: print(x))
        _add_labeled_vec3_field(self.layout, "Scale:", (-10, -10, -10), (10, 10, 10), (0, 0, 0), lambda x: print(x))
        _add_labeled_vec3_field(self.layout, "Initial position:", (-10, -10, -10), (10, 10, 10), (0, 0, 0), lambda x: print(x))
        _add_labeled_vec3_field(self.layout, "Initial velocity:", (-10, -10, -10), (10, 10, 10), (0, 0, 0), lambda x: print(x))


    def __create_object_properties_dock(self, name:str, callback_fn) -> Dock:
        pass 
    

def _create_float_field(range: tuple[float, float], default_value:float, callback)->QtWidgets.QLineEdit:
    # create validator 
    validator = QtGui.QDoubleValidator()
    validator.setRange(*range, 2)

    # create textbox and attach validator
    tbox = QtWidgets.QLineEdit()
    tbox.setText(str(default_value))
    tbox.setValidator(validator)
    tbox.textChanged.connect(callback)
    tbox.setFrame(False)
    return tbox

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


#######THE LAST VERSION OF DRAWING THE wind in areas and still remaining is changing BG color of wind to match the background but it is good
import math
import threading
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os
import time
from PyQt5.QtWidgets import (
    QApplication, 
    QMainWindow, 
    QLabel, 
    QVBoxLayout, 
    QWidget,
    QHBoxLayout,
    QFrame
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QPainter, QColor, QBrush, QPen
import sys
from config import *

class CircleWidget(QWidget):
    def __init__(self, color, size=15, parent=None):
        super().__init__(parent)
        self.color = color
        self.size = size
        self.setFixedSize(size, size)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw border (black circle)
        pen = QPen(QColor('black'), 2)
        painter.setPen(pen)
        brush = QBrush(QColor(self.color))
        painter.setBrush(brush)
        
        # Draw circle
        painter.drawEllipse(1, 1, self.size-2, self.size-2)

class EnergyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Energy Monitor")
        self.setFixedSize(350, 500)
        
        screen = QApplication.primaryScreen().geometry()
        self.move(screen.width()-380, 110)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Energy header section (battery icon and labels)
        energy_header = QWidget()
        energy_layout = QHBoxLayout(energy_header)
        
        battery_label = QLabel()
        battery_pixmap = QPixmap("battery.png").scaled(40, 40, Qt.KeepAspectRatio)
        battery_label.setPixmap(battery_pixmap)
        energy_layout.addWidget(battery_label)
        
        energy_labels = QWidget()
        energy_text_layout = QVBoxLayout(energy_labels)
        
        self.energy_label = QLabel(f"Remaining: 100%")
        self.initial_energy_label = QLabel(f"Initial: {init_energy} W")
        
        self.energy_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.initial_energy_label.setStyleSheet("font-size: 12px; color: #666;")
        
        energy_text_layout.addWidget(self.energy_label)
        energy_text_layout.addWidget(self.initial_energy_label)
        energy_layout.addWidget(energy_labels)
        
        layout.addWidget(energy_header)
        
        # Mode and cost section
        self.mode_label = QLabel("Mode: LO")
        self.mode_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.mode_label.setStyleSheet("font-size: 14px; font-weight: bold; color: blue;")
        layout.addWidget(self.mode_label)
        
        self.cost_lo_label = QLabel("Acc. Cost (LO): 0.00 W")
        self.cost_hi_label = QLabel("Acc. Cost (HI): 0.00 W")
        self.consumed_energy_label = QLabel("Consumed Energy: 0.00 W")
        
        for label in [self.cost_lo_label, self.cost_hi_label, self.consumed_energy_label]:
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            label.setStyleSheet("font-size: 12px;")
            layout.addWidget(label)
        
        # Add first separator line
        first_separator = QFrame()
        first_separator.setFrameShape(QFrame.HLine)
        first_separator.setFrameShadow(QFrame.Sunken)
        layout.addWidget(first_separator)
        
        # Position section
        self.position_label = QLabel("Position:")
        self.position_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.position_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(self.position_label)
        
        self.x_label = QLabel("X: 0.00")
        self.y_label = QLabel("Y: 0.00")
        self.z_label = QLabel("Z: 0.00")
        for label in [self.x_label, self.y_label, self.z_label]:
            label.setStyleSheet("font-size: 12px;")
            layout.addWidget(label)
        
        # Add second separator line
        second_separator = QFrame()
        second_separator.setFrameShape(QFrame.HLine)
        second_separator.setFrameShadow(QFrame.Sunken)
        second_separator.setLineWidth(2)
        layout.addWidget(second_separator)
        
        # Legend section with side-by-side layout
        legend_label = QLabel("Legend:")
        legend_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(legend_label)
        
        # Create a widget to hold both legends side by side
        legends_container = QWidget()
        legends_layout = QHBoxLayout(legends_container)
        legends_layout.setContentsMargins(0, 0, 0, 0)
        
        # === SENSORS LEGEND (LEFT SIDE) ===
        sensors_widget = QWidget()
        sensors_layout = QVBoxLayout(sensors_widget)
        sensors_layout.setContentsMargins(5, 5, 5, 5)
        
        sensors_title = QLabel("Sensors:")
        sensors_title.setStyleSheet("font-weight: bold;")
        sensors_layout.addWidget(sensors_title)
        
        # Critical sensor indicator
        critical_sensor = QWidget()
        critical_layout = QHBoxLayout(critical_sensor)
        critical_layout.setContentsMargins(0, 0, 0, 0)
        critical_circle = CircleWidget('red')
        critical_layout.addWidget(critical_circle)
        critical_text = QLabel("Critical sensor")
        critical_layout.addWidget(critical_text)
        critical_layout.addStretch()
        sensors_layout.addWidget(critical_sensor)
        
        # Normal sensor indicator
        normal_sensor = QWidget()
        normal_layout = QHBoxLayout(normal_sensor)
        normal_layout.setContentsMargins(0, 0, 0, 0)
        normal_circle = CircleWidget('black')
        normal_layout.addWidget(normal_circle)
        normal_text = QLabel("Normal sensor")
        normal_layout.addWidget(normal_text)
        normal_layout.addStretch()
        sensors_layout.addWidget(normal_sensor)
        
        # Visited sensor indicator
        visited_sensor = QWidget()
        visited_layout = QHBoxLayout(visited_sensor)
        visited_layout.setContentsMargins(0, 0, 0, 0)
        visited_circle = CircleWidget('white')
        visited_layout.addWidget(visited_circle)
        visited_text = QLabel("Visited sensor")
        visited_layout.addWidget(visited_text)
        visited_layout.addStretch()
        sensors_layout.addWidget(visited_sensor)
        
        legends_layout.addWidget(sensors_widget)
        
        # === DRONE MODE LEGEND (RIGHT SIDE) ===
        drone_mode_widget = QWidget()
        drone_mode_layout = QVBoxLayout(drone_mode_widget)
        drone_mode_layout.setContentsMargins(5, 5, 5, 5)
        
        drone_mode_title = QLabel("Drone Mode:")
        drone_mode_title.setStyleSheet("font-weight: bold;")
        drone_mode_layout.addWidget(drone_mode_title)
        
        # HI Mode indicator
        hi_mode = QWidget()
        hi_layout = QHBoxLayout(hi_mode)
        hi_layout.setContentsMargins(0, 0, 0, 0)
        hi_drone = QLabel()
        hi_drone_pixmap = QPixmap("drone.png").scaled(20, 20, Qt.KeepAspectRatio)
        hi_drone.setPixmap(hi_drone_pixmap)
        hi_drone.setStyleSheet("background-color: red; padding: 2px; border: 1px solid black;")
        hi_layout.addWidget(hi_drone)
        hi_text = QLabel("HI Mode")
        hi_layout.addWidget(hi_text)
        hi_layout.addStretch()
        drone_mode_layout.addWidget(hi_mode)
        
        # LO Mode indicator
        lo_mode = QWidget()
        lo_layout = QHBoxLayout(lo_mode)
        lo_layout.setContentsMargins(0, 0, 0, 0)
        lo_drone = QLabel()
        lo_drone_pixmap = QPixmap("drone.png").scaled(20, 20, Qt.KeepAspectRatio)
        lo_drone.setPixmap(lo_drone_pixmap)
        lo_drone.setStyleSheet("background-color: blue; padding: 2px; border: 1px solid black;")
        lo_layout.addWidget(lo_drone)
        lo_text = QLabel("LO Mode")
        lo_layout.addWidget(lo_text)
        lo_layout.addStretch()
        drone_mode_layout.addWidget(lo_mode)
        
        # Add an empty spacer widget to take up vertical space
        drone_mode_layout.addStretch()
        
        legends_layout.addWidget(drone_mode_widget)
        
        # Add the side-by-side legends container to the main layout
        layout.addWidget(legends_container)
        
        # Add a spacer at the bottom to push everything up
        layout.addStretch()
        
        layout.setSpacing(10)
        self.show()
        
    def update_energy(self, energy):
        self.energy_label.setText(f"Remaining: {int(energy)}%")
        if energy < 20:
            self.energy_label.setStyleSheet("font-size: 14px; font-weight: bold; color: red;")
        else:
            self.energy_label.setStyleSheet("font-size: 14px; font-weight: bold; color: black;")
    
    def update_mode_and_costs(self, is_hi_mode, accumulated_cost_lo, accumulated_cost_hi, consumed_energy):
        mode_text = "HI" if is_hi_mode else "LO"
        mode_color = "red" if is_hi_mode else "blue"
        self.mode_label.setText(f"Mode: {mode_text}")
        self.mode_label.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {mode_color};")
        self.cost_lo_label.setText(f"Acc. Cost (LO): {accumulated_cost_lo:.2f} W")
        self.cost_hi_label.setText(f"Acc. Cost (HI): {accumulated_cost_hi:.2f} W")
        self.consumed_energy_label.setText(f"Consumed Energy: {consumed_energy:.2f} W")
            
    def update_position(self, position):
        if position:
            self.x_label.setText(f"X: {position[0]:.2f}")
            self.y_label.setText(f"Y: {position[1]:.2f}")
            self.z_label.setText(f"Z: {position[2]:.2f}")


class DroneMonitor(threading.Thread):
    def __init__(self, simulation, window):
        super().__init__()
        self.simulation = simulation
        self.window = window
        self.running = True
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.daemon = True  # Ensures thread exits when main program exits

    def run(self):
        while self.running:
            try:
                energy = self.simulation.get_remaining_energy()
                energy=math.ceil(energy)
                
                self.window.update_energy(energy)
                
                if hasattr(self.simulation, 'is_hi_mode'):
                    self.window.update_mode_and_costs(
                        self.simulation.is_hi_mode, 
                        self.simulation.accumulated_cost_mode_LO, 
                        self.simulation.accumulated_cost_mode_HI, 
                        self.simulation.consumed_energy
                    )
                
                if self.simulation.quadcopter_handle is not None:
                    position = self.sim.getObjectPosition(self.simulation.quadcopter_handle, -1)
                    self.window.update_position(position)
                
                time.sleep(0.1)  # Reduce CPU usage
            except Exception as e:
                print(f"Error in monitor thread: {e}")
                break

    def stop(self):
        self.running = False

class DroneSimulation(QThread):
    energy_updated = pyqtSignal(float)
    
    def __init__(self, points, wind_regions):  # Changed from wind_moves to wind_regions
        super().__init__()
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.is_hi_mode = False
        self.accumulated_cost_mode_LO = 0
        self.accumulated_cost_mode_HI = 0
        self.points = points
        self.wind_icon_path = os.path.abspath('wind_icon_2.png')
        self.wind_cirle_path = os.path.abspath('wind_circle.png')
        self.wind_regions = wind_regions  # Now expects list of regions
        self.energy = 100
        self.move_count = 0
        self.quadcopter_handle = None
        self.sensors = {}
        self.wind_region_handles = []  # Stores handles for wind regions
        self.wind_icon_handles =[]
        self.consumed_energy = 0
        self.previous_sensor = None
        self.initialize_simulation()

    def initialize_simulation(self):
        self.quadcopter_handle, self.sensors = self.create_coords(self.points)
        # Create wind region visualizations
        #self.wind_icon_handle = self.create_wind_icon(region['center'])
        # self.hide_wind_icon()
        for region in self.wind_regions:
            handle = self.create_wind_region(region['center'], region['radius'])
           # icon_handle = self.create_wind_icon(region['center'])
            self.wind_region_handles.append(handle)
           # self.wind_icon_handles.append(icon_handle)
            self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 0)
           # self.sim.setObjectInt32Param(icon_handle, self.sim.objintparam_visibility_layer, 0)
        print("Simulation initialized with wind regions.")

    
    def create_wind_icon(self, center, scale=[0.3, 0.3, 0.01]):
        print('I created a wind icon')
        icon_handle = self.sim.createPrimitiveShape(
            self.sim.primitiveshape_plane, scale, 0)
        self.sim.setObjectPosition(icon_handle, -1, [center[0], center[1], 1.0])
        shape, texture_id, _ = self.sim.createTexture(self.wind_icon_path, 0, None, None)
        self.sim.setShapeTexture(
            icon_handle, texture_id, self.sim.texturemap_plane, 0, [1, 1])
        #self.sim.setObjectParent(icon_handle, self.quadcopter_handle, True)
        #self.sim.scaleObject(icon_handle, 2.0, 2.0, 1.0)
        return icon_handle
    

    # def create_wind_region(self, center, radius):
    #     """Create a horizontal circle outline for wind region visualization"""
    #     segments = 36  # Number of segments to create smooth circle
    #     height = 0.01  # Thickness of the circle

    #     # Create points for a circle
    #     points = []
    #     for i in range(segments):
    #         angle = 2 * math.pi * i / segments
    #         x = radius * math.cos(angle)
    #         y = radius * math.sin(angle)
    #         points.extend([x, y, 0])  # First point of line segment
    #         angle = 2 * math.pi * (i + 1) / segments
    #         x = radius * math.cos(angle)
    #         y = radius * math.sin(angle)
    #         points.extend([x, y, 0])  # Second point of line segment

    #     # Create the circle outline using a compound line object
    #     circle_handle = self.sim.addDrawingObject(
    #         objectType=self.sim.drawing_lines,
    #         size=height,
    #         color=[0, 0.7, 0, 0.5],  # Semi-transparent green
    #         duplicateTolerance=0,
    #         parentObjectHandle=-1,
    #         maxElements=segments*2  # Two points per segment
    #     )
        
    #     # Add the line segments to create the circle
    #     self.sim.addDrawingObjectItems(circle_handle, points)
        
    #     # Position the circle at the specified center
    #     self.sim.setObjectPosition(circle_handle, -1, [center[0], center[1], 0.1])
        
    #     return circle_handle


    def create_wind_region(self, center, radius):
        print(f"creted wind regionwith center: {center}")
        """Create a horizontal circle outline with wind icon using a thin cylinder"""
        # Create a thin cylinder for the perimeter
        cylinder = self.sim.createPrimitiveShape(
            self.sim.primitiveshape_plane,
            [radius*2,radius*2, 0.02],  # Explicitly set radii and thin height
            0
        )





        self.sim.setObjectPosition(cylinder, -1, [center[0], center[1], 0.9])
        #shape, texture_id, _ = self.sim.createTexture(self.wind_cirle_path, 0,  [radius*2, radius*2], None)
        shape, texture_id, _ = self.sim.createTexture(self.wind_cirle_path, 0,  None, None)
        self.sim.setShapeTexture(
            cylinder, texture_id, self.sim.texturemap_plane, 0, [radius*2,radius*2])
        #self.sim.setObjectParent(icon_handle, self.quadcopter_handle, True)
        #self.sim.scaleObject(cylinder, 2.0, 2.0, 1.0)
        return cylinder







        
    #     # Rotate cylinder to lie flat on XY plane
    #     self.sim.setObjectOrientation(cylinder, -1, [math.pi/2, 0, 0])
        
    #     # Position it at ground level
    #     self.sim.setObjectPosition(cylinder, -1, [center[0], center[1], 0.9])
    #     self.sim.setObjectOrientation(cylinder, -1, [0, 0, 0])  # No rotation needed
        
    #     # Remove top/bottom faces (keep only the side)
    #     self.sim.setShapeColor(cylinder, None, self.sim.colorcomponent_auxiliary, [0, 0, 0])
        
    #     # # Apply wind icon texture to the side
    #     # shape, texture_id, _ = self.sim.createTexture(self.wind_icon_path, 0, None, None)
    #     # self.sim.setShapeTexture(
    #     #     cylinder,
    #     #     texture_id,
    #     #     self.sim.texturemap_cylinder,
    #     #     0,
    #     #     [1, 1]
    #     # )
    #     darker_green = [0.2, 0.5, 0.3]
    #     # Make it semi-transparent
    #     # self.sim.setShapeColor(cylinder, None, self.sim.colorcomponent_transparency, darker_green)
    #     self.sim.setShapeColor(cylinder, None, 0, darker_green)
    #    # self.sim.setShapeColor(cylinder, None, self.sim.colorcomponent_transparency, [0.5])
        
        
    #     # Initialize as hidden
    #     self.sim.setObjectInt32Param(cylinder, self.sim.objintparam_visibility_layer, 0)
        # return cylinder

    def calculate_intersection(self, start, end, center, radius):
        """Calculate distance through a circular region"""
        x1, y1 = start
        x2, y2 = end
        cx, cy = center
        
        dx = x2 - x1
        dy = y2 - y1
        a = dx**2 + dy**2
        if a == 0: return 0.0
        
        b = 2 * (dx*(x1 - cx) + dy*(y1 - cy))
        c = (x1 - cx)**2 + (y1 - cy)**2 - radius**2
        disc = b**2 - 4*a*c
        
        if disc < 0: return 0.0
        
        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2*a)
        t2 = (-b + sqrt_disc) / (2*a)
        t_start = max(0.0, min(t1, t2))
        t_end = min(1.0, max(t1, t2))
        
        if t_start >= t_end: return 0.0
        return (t_end - t_start) * math.hypot(dx, dy)

    def get_consumed_energy(self):
        return self.consumed_energy

    def get_remaining_energy(self):
        return self.energy

    def is_in_wind_region(self, position, move_count):
        for region in self.wind_regions:
            if move_count in region["moves"]:
                distance = math.sqrt((position[0] - region["center"][0])**2 + (position[1] - region["center"][1])**2)
                if distance <= region["radius"]:
                    return True
        return False

    def calculate_energy_consumption(self, base_energy, start_pos, end_pos, move_count):
        total_distance = math.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)
        wind_distance = 0
        no_wind_distance = total_distance

        for region in self.wind_regions:
            if move_count in region["moves"]:
                # Calculate the intersection of the drone's path with the wind region
                # This is a simplified calculation; you may need a more accurate method
                distance = math.sqrt((end_pos[0] - region["center"][0])**2 + (end_pos[1] - region["center"][1])**2)
                if distance <= region["radius"]:
                    wind_distance += total_distance
                    no_wind_distance -= total_distance

        energy_consumed = (no_wind_distance * coef_energy_no_wind_real) + (wind_distance * coef_energy_wind_real)
        return energy_consumed

    def create_label(self, object_handle, label_text, is_critical=False):
        try:
            object_position = self.sim.getObjectPosition(object_handle, -1)
            color = [1, 0, 0] if is_critical else [0, 0, 0]
            
            text_shape = self.sim.generateTextShape(
                label_text, color, 0.15, True
            )
            label_position = [
                object_position[0] + 0.07,
                object_position[1] + 0.07,
                object_position[2] + 0.01
            ]
            self.sim.setObjectPosition(text_shape, -1, label_position)
        except Exception as e:
            print(f"Error creating label for object {object_handle}: {e}")

    def create_plane(self, center_x, center_y, width, height):
        # Create a smaller plane by reducing the width and height
        plane_handle = self.sim.createPrimitiveShape(
            self.sim.primitiveshape_cuboid,  
            [width, height, 0.01],  # Keep the original dimensions from parameters
            0
        )
        self.sim.setObjectPosition(plane_handle, -1, [center_x, center_y, -0.005])
        #lighter green [0.2, 0.6, 0.2]
        #darker [0.2, 0.5, 0.2]
        # Set darker green color (RGB values between 0 and 1)
        darker_green = [0.2, 0.68, 0.2]  # Much darker shade of green
        self.sim.setShapeColor(plane_handle, None, 0, darker_green)
        
        return plane_handle
    def create_bluetooth_icon(self, position, is_critical):
        # Create a small plane for the bluetooth icon
        icon_scale = [0.3, 0.3, 0.3]  # Smaller scale for bluetooth icon
        icon_handle = self.sim.createPrimitiveShape(
            self.sim.primitiveshape_plane, icon_scale, 0
        )
        
        # Position the icon slightly above the sensor
        icon_position = [
            position[0]+0.4,
            position[1]+0.4,
            position[2] + 0.2  # Slightly above the sensor
        ]
        self.sim.setObjectPosition(icon_handle, -1, icon_position)
        
        # Choose icon based on whether the sensor is critical
        icon_path = os.path.abspath('bluetooth_red.png' if is_critical else 'bluetooth_black.png')
        shape, texture_id, _ = self.sim.createTexture(icon_path, 0, None, None)
        self.sim.setShapeTexture(
            icon_handle, texture_id, self.sim.texturemap_plane, 0, [1, 1]
        )
        #darker_green = [0.2, 0.5, 0.2]
        #self.sim.setShapeColor(icon_handle, None, 0, darker_green)
        return icon_handle

    def create_quadcopter(self, position):
        quadcopter_handle = self.sim.loadModel('models/robots/mobile/Quadcopter.ttm')
        self.sim.setObjectPosition(quadcopter_handle, -1, position)
        return quadcopter_handle

    def create_vision_sensor(self, position):
        options = 0 | 4
        intParams = [256, 256, 0, 0]
        floatParams = [0.01, 10.0, 60.0 * (math.pi / 180), 0.05,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        sensor_handle = self.sim.createVisionSensor(options, intParams, floatParams)
        self.sim.setObjectPosition(sensor_handle, -1, position)
        return sensor_handle

    def remove_default_floor(self):
        try:
            floor_handle = self.sim.getObject('/Floor')
            box_handle = self.sim.getObject('/Floor/box')
            self.sim.removeObjects([floor_handle, box_handle], False)
        except Exception:
            pass

    def resize_existing_floor(self, sensor_positions):
        self.remove_default_floor()
        min_x = min(pos[0] for pos in sensor_positions)
        max_x = max(pos[0] for pos in sensor_positions)
        min_y = min(pos[1] for pos in sensor_positions)
        max_y = max(pos[1] for pos in sensor_positions)
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        width = max_x - min_x + 2
        height = max_y - min_y + 2
        # Create a smaller plane by adding less padding
        plane_handle = self.create_plane(center_x, center_y, width + 4, height + 4)  # Reduced padding from 10 to 4
        return plane_handle

    def create_coords(self, points):
        # Create floor based on all points
        sensor_positions = [(data["x"], data["y"]) for key, data in points.items()]
        self.resize_existing_floor(sensor_positions)
        
        # Create quadcopter at position B
        quadcopter_position = [points["B"]["x"], points["B"]["y"], 2]
        quadcopter_handle = self.create_quadcopter(quadcopter_position)
        
        # Create sensors and bluetooth icons for all points
        sensors = {}
        for key, data in points.items():
            sensor_position = [data["x"], data["y"], 1]
            sensor_handle = self.create_vision_sensor(sensor_position)
            self.sim.setObjectAlias(sensor_handle, key)
            
            is_critical = data.get('c', False)
            label_shape=self.create_label(sensor_handle, key, is_critical)
            
            # Create bluetooth icon for each sensor
            bluetooth_handle = self.create_bluetooth_icon(sensor_position, is_critical)
            # Optionally store the bluetooth handle if you need to modify it later
            sensors[key] = {
                'sensor': sensor_handle,
                'bluetooth': bluetooth_handle,
                'label':label_shape
            }

        return quadcopter_handle, sensors



    # def move_drone_to_sensor(self, sensor_name, is_HI_Mode, acc_cost_hi, acc_cost_lo):
    #     time.sleep(0.5)
    #     self.move_count += 1
        
    #     # Update wind region visibility
    #     for i, region in enumerate(self.wind_regions):
    #             handle = self.wind_region_handles[i]
    #             if self.move_count in region['moves']:
    #                 self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 1)
    #                 #self.sim.setObjectInt32Param(self.wind_icon_handles[i], self.sim.objintparam_visibility_layer, 1)

    #             else:
    #                 self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 0)
    #                # self.sim.setObjectInt32Param(self.wind_icon_handles[i], self.sim.objintparam_visibility_layer, 0)
    #     # Calculate path energy consumption
    #     prev_point = self.points.get(self.previous_sensor, self.points["B"])
    #     current_point = self.points[sensor_name]
    #     start = [prev_point['x'], prev_point['y']]
    #     end = [current_point['x'], current_point['y']]
    #     total_dist = math.hypot(end[0]-start[0], end[1]-start[1])
    #     print(f"moving from point {prev_point} to {current_point} and distance ={total_dist}")
        
    #     wind_dist = 0.0
    #     for region in self.wind_regions:
    #         if self.move_count in region['moves']:
    #             wind_dist += self.calculate_intersection(
    #                 start, end, 
    #                 region['center'], 
    #                 region['radius']
    #             )
        
    #     energy_used = (wind_dist * coef_energy_wind_real + 
    #                   (total_dist - wind_dist) * coef_energy_no_wind_real)
        
        
    #     self.consumed_energy += energy_used
    #     self.energy -= (energy_used / init_energy * 100)
    #     self.energy_updated.emit(self.energy)

    #     # Rest of movement logic remains the same
    #     target_pose = self.sim.getObjectPose(self.sensors[sensor_name]['sensor'], -1)
    #     target_pose[2] += 1.0
    #     self.sim.moveToPose({
    #         'object': self.quadcopter_handle,
    #         'targetPose': target_pose,
    #         'maxVel': [0.05]*4,
    #         'maxAccel': [0.05]*4,
    #         'maxJerk': [0.1]*4,
    #         'relativeTo': -1
    #     })
    #     self.previous_sensor = sensor_name

    #     #A Afficher sur le widget après la mise à jour de ton energy consumed !! 
    #     print("is_HI_Mode_simu", is_HI_Mode)
    #     print("Move to sensor : ", sensor_name)
    #     print("accumulated_cost_mode_LO", acc_cost_lo)
    #     print("accumulated_cost_mode_HI", acc_cost_hi)
    #     print("real_consumed_energy",self.consumed_energy)
    #     print("real_remaining_energy",self.energy)




    def mark_label_visited(self, sensor_name, is_critical): ##COLORING
        try:
            print(f"/{sensor_name}[1]/text")
            text_shape = self.sim.getObject(f"/{sensor_name}[1]/text")
           
            # Soft light red for critical sensors
            soft_red = [1, 1, 1]  
            # Soft gray for non-critical sensors
            soft_gray = [1, 1, 1]
            
            color = soft_red if is_critical else soft_gray
            
            # Update text color
            # self.sim.generateTextShape(
            #     self.sim.getObjectName(text_shape), 
            #     color, 
            #     0.15, 
            #     True, 
            #     text_shape  # Reuse existing text shape
            # )
            self.sim.setObjectColor(text_shape, 0, self.sim.colorcomponent_ambient_diffuse, color)
            #self.sim.setShapeColor(text_shape, None, 0, color)
        except Exception as e:
            print(f"Error marking label visited: {e}")
    def mark_label_visited_text(self, sensor_name, is_critical): ##COLORING
        try:
            print(f"/{sensor_name}[1]/")
            text_shape = self.sim.getObject(f"/{sensor_name}[1]")
            text_shape_inner_text = self.sim.getObject(f"/{sensor_name}[1]/text")
            print ("the object i got: ",text_shape )
            object_position = self.sim.getObjectPosition(text_shape, -1)
            self.sim.removeObject(text_shape)
            self.sim.removeObject(text_shape_inner_text)
            visited_string=sensor_name+ " v"
            # Soft light red for critical sensors
            
            color = [1, 0, 0] if is_critical else [0, 0, 0]
            
            text_shape = self.sim.generateTextShape(
                visited_string, color, 0.17, True
            )
            label_position = [
                object_position[0] + 0.0,
                object_position[1] + 0.0,
                object_position[2] + 0.0
            ]
            self.sim.setObjectPosition(text_shape, -1, label_position)
        except Exception as e:
            print(f"Error marking label visited_2: {e}")
    def color_drone_mode_if_HI(self,isHi):
        drone_circle_shape = self.sim.getObject("/Quadcopter/base/target")
        soft_red = [0.8, 0.0, 0.0]  
        # Soft gray for non-critical sensors
        soft_gray = [0.0, 0.1, 0.9]
        color = soft_red if isHi else soft_gray
        self.sim.setObjectColor(drone_circle_shape, 0, self.sim.colorcomponent_ambient_diffuse, color)




    def move_drone_to_sensor(self, sensor_name, is_HI_Mode, acc_cost_hi, acc_cost_lo):
        self.color_drone_mode_if_HI(is_HI_Mode)
        time.sleep(0.5)
        self.move_count += 1
        self.is_hi_mode = is_HI_Mode
        self.accumulated_cost_mode_LO = acc_cost_lo
        self.accumulated_cost_mode_HI = acc_cost_hi
        # Update wind region visibility
        for i, region in enumerate(self.wind_regions):
            handle = self.wind_region_handles[i]
            if self.move_count in region['moves']:
                self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 1)
            else:
                self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 0)

        # Calculate path energy consumption
        prev_point = self.points.get(self.previous_sensor, self.points["B"])
        current_point = self.points[sensor_name]
        start = [prev_point['x'], prev_point['y']]
        end = [current_point['x'], current_point['y']]
        total_dist = math.hypot(end[0]-start[0], end[1]-start[1])
        
        wind_dist = 0.0
        active_regions = []
        for region in self.wind_regions:
            if self.move_count in region['moves']:
                region_dist = self.calculate_intersection(start, end, region['center'], region['radius'])
                
                print("calculate_intersection returned: ",region_dist)
                if region_dist > 0:
                    wind_dist += region_dist
                    active_regions.append({
                        'center': region['center'],
                        'radius': region['radius'],
                        'distance': region_dist
                    })

        energy_used = (wind_dist * coef_energy_wind_real + 
                    (total_dist - wind_dist) * coef_energy_no_wind_real)
        
        # Add detailed print output
        print(f"\n=== Move {self.move_count} to {sensor_name} ===")
        print(f"Total distance: {total_dist:.2f} units")
        if wind_dist > 0:
            print(f"Passed through {len(active_regions)} wind region(s):")
            for i, region in enumerate(active_regions, 1):
                print(f"  Region {i}: Center {region['center']}, Radius {region['radius']}")
                print(f"    Wind distance: {region['distance']:.2f} units")
            print(f"Total wind distance: {wind_dist:.2f} units")
            print(f"Normal distance: {total_dist - wind_dist:.2f} units")
        else:
            print("No wind regions encountered")
        
        print(f"Energy cost calculation:")
        print(f"  Wind distance ({wind_dist:.2f}) * {coef_energy_wind_real} = {wind_dist * coef_energy_wind_real:.2f}W")
        print(f"  Normal distance ({total_dist - wind_dist:.2f}) * {coef_energy_no_wind_real} = {(total_dist - wind_dist) * coef_energy_no_wind_real:.2f}W")
        print(f"Total energy cost: {energy_used:.2f}W")
        
        self.consumed_energy += energy_used
        self.energy -= (energy_used / init_energy * 100)
        self.energy_updated.emit(self.energy)

        # Rest of movement logic
        target_pose = self.sim.getObjectPose(self.sensors[sensor_name]['sensor'], -1)
        target_pose[2] += 1.0
        self.sim.moveToPose({
            'object': self.quadcopter_handle,
            'targetPose': target_pose,
            'maxVel': [0.05]*4,
            'maxAccel': [0.05]*4,
            'maxJerk': [0.1]*4,
            'relativeTo': -1
        })

        sensor_data = self.sensors.get(sensor_name)
        text_shape = sensor_data['label']
        is_critical = self.points[sensor_name].get('c', False)
        
        # Mark label as visited
        #self.mark_label_visited(text_shape, is_critical)
        self.mark_label_visited(sensor_name, is_critical)
        
        self.previous_sensor = sensor_name
        print("is_HI_Mode_simu", is_HI_Mode)
        print("Move to sensor : ", sensor_name)
        print("accumulated_cost_mode_LO", acc_cost_lo)
        print("accumulated_cost_mode_HI", acc_cost_hi)
        print("real_consumed_energy",self.consumed_energy)
        print("real_remaining_energy",self.energy)
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap
import sys

class EnergyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Energy Monitor")
        self.setFixedSize(200, 180)
        
        # Set window position from left and bottom
        screen = QApplication.primaryScreen().geometry()
        self.move(screen.width()-230, 110)  # 30px from left, 80px from bottom
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Add battery image
        battery_label = QLabel()
        battery_pixmap = QPixmap("battery.png")
        battery_pixmap = battery_pixmap.scaled(50, 50, Qt.KeepAspectRatio)
        battery_label.setPixmap(battery_pixmap)
        battery_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(battery_label)
        
        # Add energy label
        self.energy_label = QLabel("Energy: 100%")
        self.energy_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.energy_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(self.energy_label)
        
        # Add some spacing between image and text
        layout.setSpacing(10)
        
        self.show()
    
    def update_energy(self, energy):
        self.energy_label.setText(f"Energy: {int(energy)}%")
        if energy < 20:
            self.energy_label.setStyleSheet("font-size: 16px; font-weight: bold; color: red;")
        else:
            self.energy_label.setStyleSheet("font-size: 16px; font-weight: bold; color: black;")

class DroneSimulation(QThread):
    energy_updated = pyqtSignal(float)
    
    def __init__(self, points, wind_moves):
        super().__init__()
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.points = points
        self.wind_icon_path = os.path.abspath('wind_icon_2.png')
        self.wind_moves = wind_moves
        self.energy = 100
        self.previous_energy = 100
        self.move_count = 0
        self.quadcopter_handle = None
        self.sensors = {}
        self.wind_icon_handle = None
        self.consumed_energy = 0
        self.previous_sensor = None
        self.initialize_simulation()

    def initialize_simulation(self):
        self.quadcopter_handle, self.sensors = self.create_coords(self.points)
        drone_pos = self.sim.getObjectPosition(self.quadcopter_handle, -1)
        self.wind_icon_handle = self.create_wind_icon([
            drone_pos[0] + 0.6,
            drone_pos[1]+0.6,
            drone_pos[2]
        ])
        self.hide_wind_icon()
        print("Simulation initialized successfully.")

    def get_energy(self):
        return self.consumed_energy

    def calculate_energy_consumption(self, base_energy, wind_factor):
        return base_energy * (1 + wind_factor)

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
        plane_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_plane, [1, 1, 0.01], 0)
        self.sim.setObjectPosition(plane_handle, -1, [center_x, center_y, 0])
        self.sim.scaleObject(plane_handle, width, height, 1, 0)
        return plane_handle

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
        plane_handle = self.create_plane(center_x, center_y, width+10, height+10)
        return plane_handle

    def create_coords(self, points):
        sensor_positions = [(data["x"], data["y"]) for key, data in points.items()]
        self.resize_existing_floor(sensor_positions)
        
        # Create quadcopter at position B
        quadcopter_position = [points["B"]["x"], points["B"]["y"], 2]
        quadcopter_handle = self.create_quadcopter(quadcopter_position)
        
        # Create sensors for all points, including B
        sensors = {}
        for key, data in points.items():
            sensor_position = [data["x"], data["y"], 1]
            sensor_handle = self.create_vision_sensor(sensor_position)
            self.sim.setObjectAlias(sensor_handle, key)
            is_critical = data.get('c', False)
            self.create_label(sensor_handle, key, is_critical)
            sensors[key] = sensor_handle

        return quadcopter_handle, sensors

    def create_wind_icon(self, position, scale=[0.3, 0.3, 0.01]):
        icon_handle = self.sim.createPrimitiveShape(
            self.sim.primitiveshape_plane, scale, 0)
        self.sim.setObjectPosition(icon_handle, -1, position)
        shape, texture_id, _ = self.sim.createTexture(self.wind_icon_path, 0, None, None)
        self.sim.setShapeTexture(
            icon_handle, texture_id, self.sim.texturemap_plane, 0, [1, 1])
        self.sim.setObjectParent(icon_handle, self.quadcopter_handle, True)
        self.sim.scaleObject(icon_handle, 2.0, 2.0, 1.0)
        return icon_handle

    def show_wind_icon(self):
        if self.wind_icon_handle:
            self.sim.setObjectInt32Param(
                self.wind_icon_handle,
                self.sim.objintparam_visibility_layer,
                1
            )

    def hide_wind_icon(self):
        if self.wind_icon_handle:
            self.sim.setObjectInt32Param(
                self.wind_icon_handle,
                self.sim.objintparam_visibility_layer,
                0
            )

    def move_drone_to_sensor(self, sensor_name):
        self.move_count += 1
        base_energy = 1
        wind_factor = 2 if self.move_count in self.wind_moves else 0

        if self.quadcopter_handle == -1:
            raise ValueError("Drone not found in the scene.")

        sensor_handle = self.sensors.get(sensor_name)
        if sensor_handle is None:
            raise ValueError(f"Sensor '{sensor_name}' not found.")

        if self.previous_sensor is None:
            prev_point = self.points["B"]
        else:
            prev_point = self.points[self.previous_sensor]

        current_point = self.points[sensor_name]
        distance = math.sqrt(
            (current_point["x"] - prev_point["x"])**2 +
            (current_point["y"] - prev_point["y"])**2
        )
        self.previous_energy = self.energy
        energy_consumed = self.calculate_energy_consumption(base_energy, wind_factor)
        self.energy -= energy_consumed
        self.consumed_energy += energy_consumed * distance
        
        self.energy_updated.emit(self.energy)

        if wind_factor > 0:
            self.show_wind_icon()
        else:
            self.hide_wind_icon()

        target_pose = self.sim.getObjectPose(sensor_handle, -1)
        target_pose[2] += 1.0
        
        params = {
            'object': self.quadcopter_handle,
            'targetPose': target_pose,
            'maxVel': [0.05, 0.05, 0.05, 0.05],
            'maxAccel': [0.05, 0.05, 0.05, 0.05],
            'maxJerk': [0.1, 0.1, 0.1, 0.1],
            'relativeTo': -1
        }
        
        self.sim.moveToPose(params)
        self.previous_sensor = sensor_name

    def run(self):
        sensors_to_visit = ['C1', 'C2', 'C3', 'C4', 'C1', 'C2', 'B']
        for sensor_name in sensors_to_visit:
            self.move_drone_to_sensor(sensor_name)
            time.sleep(0.5)

def main():
    points = {
        "B": {"x": 5, "y": 7},
        "C1": {"x": 1, "y": 5, "c": True},
        "C2": {"x": 2, "y": 1, "c": False},
        "C3": {"x": 6, "y": 2, "c": True},
        "C4": {"x": 10, "y": 6, "c": False},
    }
    wind_moves = [4, 6]

    app = QApplication(sys.argv)
    energy_window = EnergyWindow()
    
    simulation = DroneSimulation(points, wind_moves)
    simulation.energy_updated.connect(energy_window.update_energy)
    simulation.start()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
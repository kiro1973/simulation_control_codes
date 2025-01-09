import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os

class DroneSimulation:
    def __init__(self, points, wind_moves):
        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.points = points
        self.wind_icon_path =  os.path.abspath('wind_icon_2.png')  # Replace with your image path
        self.wind_moves = wind_moves  # Moves with wind effect
        self.energy = 100  # Starting energy at 100%
        self.move_count = 0  # To track the number of moves
        self.quadcopter_handle = None
        self.sensors = {}
        self.wind_icon_handle = None
        self.initialize_simulation()

    def initialize_simulation(self):
        self.quadcopter_handle, self.sensors = self.create_coords(self.points)
        self.wind_icon_handle = self.create_textured_icon(self.wind_icon_path, position=[0, 0, 2])
        # Initially hide the wind icon
        self.hide_wind_icon()
        print("Simulation initialized.")

    def get_energy(self):
        """
        Returns the remaining energy as a percentage.
        """
        return self.energy

    def calculate_energy_consumption(self, base_energy, wind_factor):
        """
        Calculate energy consumption based on wind factor.
        """
        return base_energy * (1 + wind_factor)

    def create_label(self, object_handle, label_text):
        """
        Create a label for an object in the scene.
        """
        try:
            object_position = self.sim.getObjectPosition(object_handle, -1)
            text_shape = self.sim.generateTextShape(
                label_text, [0, 0, 0], 0.15, True
            )
            label_position = [
                object_position[0] + 0.07,
                object_position[1] + 0.07,
                object_position[2] + 0.01
            ]
            self.sim.setObjectPosition(text_shape, -1, label_position)
            print(f"Label '{label_text}' created for object {object_handle}")
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
            print("Default floor removed successfully.")
        except Exception as e:
            print(f"Warning: Default floor 'Floor' not found or could not be removed. Error: {e}")

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
        plane_handle = self.sim.createPrimitiveShape(
            self.sim.primitiveshape_plane, [1, 1, 0.01], 0)
        self.sim.setObjectPosition(plane_handle, -1, [center_x, center_y, 0])
        self.sim.scaleObject(plane_handle, width, height, 1, 0)
        print(
            f"Created new resizable plane: width={width}, height={height}, center=({center_x}, {center_y})")
        return plane_handle

    def create_coords(self, points):
        points_copy = points.copy()
        quadcopter_data = points_copy.pop("B")
        quadcopter_position = [quadcopter_data["x"], quadcopter_data["y"], 2]
        sensor_positions = [
            (data["x"], data["y"]) for key, data in points_copy.items()]
        self.resize_existing_floor(
            sensor_positions + [(quadcopter_position[0], quadcopter_position[1])])
        quadcopter_handle = self.create_quadcopter(quadcopter_position)

        sensors = {}
        for key, data in points_copy.items():
            sensor_position = [data["x"], data["y"], 1]
            sensor_handle = self.create_vision_sensor(sensor_position)
            self.sim.setObjectAlias(sensor_handle, key)
            self.create_label(sensor_handle, key)
            sensors[key] = sensor_handle

        return quadcopter_handle, sensors

    def create_textured_icon(self, image_path, position, scale=[1.0, 1.0, 0.01]):
        """
        Create a textured plane to display an external image as an icon.
        """
        # Create a flat plane
        icon_handle = self.sim.createPrimitiveShape(
            self.sim.primitiveshape_plane, scale, 0)
        # Set the plane's position
        self.sim.setObjectPosition(icon_handle, -1, position)
        # Create the texture
        shape, texture_id, _ = self.sim.createTexture(image_path, 0, None, None)
        self.sim.setShapeTexture(
            icon_handle, texture_id, self.sim.texturemap_plane, 0, [1, 1])
        print(
            f"Textured icon created with image '{image_path}' at position {position}")
        return icon_handle

    def show_wind_icon(self):
        """
        Show the wind icon by enabling its visibility.
        """
        if self.wind_icon_handle:
            self.sim.setObjectInt32Param(self.wind_icon_handle, self.sim.objintparam_visibility_layer, 1)
            print("Wind icon shown.")

    def hide_wind_icon(self):
        """
        Hide the wind icon by disabling its visibility.
        """
        if self.wind_icon_handle:
            self.sim.setObjectInt32Param(self.wind_icon_handle, self.sim.objintparam_visibility_layer, 0)
            print("Wind icon hidden.")

    def move_drone_to_sensor(self, sensor_name):
        """
        Move the drone (Quadcopter) to the position of the specified sensor.
        """
        self.move_count += 1
        base_energy = 2  # Base energy consumption per move
        wind_factor = 1 if self.move_count in self.wind_moves else 0  # Wind doubles energy consumption

        drone_handle = self.quadcopter_handle
        if drone_handle == -1:
            raise ValueError("Drone 'Quadcopter' not found in the scene.")

        sensor_handle = self.sensors.get(sensor_name)
        if sensor_handle is None:
            raise ValueError(f"Sensor '{sensor_name}' not found in the scene.")

        sensor_position = self.sim.getObjectPose(sensor_handle, -1)
        sensor_position[2] += 1.0

        # Update energy consumption
        energy_consumed = self.calculate_energy_consumption(base_energy, wind_factor)
        self.energy -= energy_consumed

        # Show or hide the wind icon based on wind factor
        if wind_factor > 0:
            self.show_wind_icon()
        else:
            self.hide_wind_icon()

        # Move the drone to the target position
        params = {
            'object': drone_handle,
            'targetPose': sensor_position,
            'maxVel': [0.1, 0.1, 0.1, 0.1],
            'maxAccel': [0.05, 0.05, 0.05, 0.05],
            'maxJerk': [0.1, 0.1, 0.1, 0.1],
            'relativeTo': -1
        }
        self.sim.moveToPose(params)

        print(f"Drone moved to {sensor_position[:3]}, energy remaining: {self.get_energy()}%")

if __name__ == "__main__":
    points = {
        "B": {"x": 5, "y": 7},
        "C1": {"x": 1, "y": 5, "c": True},
        "C2": {"x": 2, "y": 1, "c": False},
        "C3": {"x": 6, "y": 2, "c": True},
        "C4": {"x": 10, "y": 6, "c": False},
    }

    # Get absolute path for the texture file
    

    # Moves where wind is present
    wind_moves = [4, 5]  # The 4th and 6th moves will have wind

    # Initialize the simulation
    simulation = DroneSimulation(points, wind_moves)

    # Define the sequence of sensors to visit
    sensors_to_visit = ['C1', 'C2', 'C3', 'C4', 'C1', 'C2']

    # Move the drone to each sensor
    for sensor_name in sensors_to_visit:
        simulation.move_drone_to_sensor(sensor_name)
    

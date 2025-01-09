import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Connect to CoppeliaSim
client = RemoteAPIClient()
sim = client.getObject('sim')

def create_label(object_handle, label_text):
    """
    Create a label for an object in the scene.

    :param object_handle: Handle of the object to label
    :param label_text: Text of the label
    """
    try:
        # Get the position of the object
        object_position = sim.getObjectPosition(object_handle, -1)

        # Generate the text shape for the label
        text_shape = sim.generateTextShape(
            label_text,        # Text to display
            [0, 0, 0],         # Color (white)
            0.15,               # Text height
            True               # Center the text
        )

        # Place the label slightly above the object
        label_position = [
            object_position[0] + 0.07,
            object_position[1]+ 0.07,
            object_position[2] + 0.01
        ]
        sim.setObjectPosition(text_shape, -1, label_position)

        print(f"Label '{label_text}' created for object {object_handle}")
    except Exception as e:
        print(f"Error creating label for object {object_handle}: {e}")


def create_plane(center_x, center_y, width, height):
    plane_handle = sim.createPrimitiveShape(sim.primitiveshape_plane, [1, 1, 0.01], 0)
    sim.setObjectPosition(plane_handle, -1, [center_x, center_y, 0])
    sim.scaleObject(plane_handle, width, height, 1, 0)
    return plane_handle

def create_quadcopter(position):
    quadcopter_handle = sim.loadModel('models/robots/mobile/Quadcopter.ttm')
    sim.setObjectPosition(quadcopter_handle, -1, position)
    return quadcopter_handle

def create_vision_sensor(position):
    options = 0 | 4
    intParams = [256, 256, 0, 0]
    floatParams = [0.01, 10.0, 60.0 * (3.14159 / 180), 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    sensor_handle = sim.createVisionSensor(options, intParams, floatParams)
    sim.setObjectPosition(sensor_handle, -1, position)
    return sensor_handle

def remove_default_floor():
    try:
        floor_handle = sim.getObject('/Floor')
        box_handle = sim.getObject('/Floor/box')
        sim.removeObjects([floor_handle, box_handle], False)
        print("Default floor removed successfully.")
    except Exception as e:
        print(f"Warning: Default floor 'Floor' not found or could not be removed. Error: {e}")

def resize_existing_floor(sensor_positions):
    remove_default_floor()
    min_x = min(pos[0] for pos in sensor_positions)
    max_x = max(pos[0] for pos in sensor_positions)
    min_y = min(pos[1] for pos in sensor_positions)
    max_y = max(pos[1] for pos in sensor_positions)
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    width = max_x - min_x + 2
    height = max_y - min_y + 2
    plane_handle = sim.createPrimitiveShape(sim.primitiveshape_plane, [1, 1, 0.01], 0)
    sim.setObjectPosition(plane_handle, -1, [center_x, center_y, 0])
    sim.scaleObject(plane_handle, width, height, 1, 0)
    print(f"Created new resizable plane: width={width}, height={height}, center=({center_x}, {center_y})")
    return plane_handle

def create_coords(pointss):
    points=pointss.copy()
    print("copied points dict"+ str(points))
    quadcopter_data = points.pop("B")
    quadcopter_position = [quadcopter_data["x"], quadcopter_data["y"], 2]
    sensor_positions = [(data["x"], data["y"]) for key, data in points.items()]
    resize_existing_floor(sensor_positions + [(quadcopter_position[0], quadcopter_position[1])])
    # Create quadcopter
    quadcopter_handle = create_quadcopter(quadcopter_position)
    

    # Create sensors with names
    sensor_handles = []
    for key, data in points.items():
        sensor_position = [data["x"], data["y"], 1]
        sensor_handle = create_vision_sensor(sensor_position)

        # Assign the dictionary key as the sensor's name
        sim.setObjectAlias(sensor_handle, key)

        # Add a label for the sensor
        create_label(sensor_handle, key)

        # Add the sensor handle to the list
        sensor_handles.append(sensor_handle)

    return quadcopter_handle, sensor_handles
# Main execution


def move_drone_to_sensor(sensor_name):
    """
    Move the drone (Quadcopter) to the position of the specified sensor using sim.moveToPose.

    :param sensor_name: Name of the sensor object in the scene (e.g., 'C1').
    """
    # Set motion parameters for very slow and smooth motion
    maxVel = [0.03, 0.03, 0.03, 0.03]   # Extremely low velocity
    maxAccel = [0.02, 0.02, 0.02, 0.02] # Extremely low acceleration
    maxJerk = [0.1, 0.1, 0.1, 0.1]      # Lower jerk for smooth transitions

    try:
        # Get the handle for the drone (Quadcopter)
        drone_handle = sim.getObject('/Quadcopter')
        if drone_handle == -1:
            raise ValueError("Drone 'Quadcopter' not found in the scene.")

        # Get the handle for the sensor
        sensor_handle = sim.getObject('/' + sensor_name + '[0]')
        if sensor_handle == -1:
            raise ValueError(f"Sensor '{sensor_name}' not found in the scene.")

        # Get the position of the sensor
        sensor_position = sim.getObjectPose(sensor_handle, -1)

        # Adjust the height to hover slightly above the sensor
        sensor_position[2] += 1.0
        print("sensor_position:",sensor_position,"*******")
        # Define motion parameters
        params = {
            'object': drone_handle,
            'targetPose': sensor_position,
            'maxVel': maxVel,
            'maxAccel': maxAccel,
            'maxJerk': maxJerk,
            'relativeTo': -1  # World frame
        }

        # Move the drone to the target pose
        sim.moveToPose(params)
        print(f"Drone successfully moved to the position of sensor '{sensor_name}' with slow and smooth motion.")

    except Exception as e:
        print(f"Error moving drone to sensor '{sensor_name}': {e}")




if __name__ == "__main__":
    points = {
        "B": {"x": 5, "y": 7},
        "C1": {"x": 1, "y": 5, "c": True},
        "C2": {"x": 2, "y": 1, "c": False},
        "C3": {"x": 6, "y": 2, "c": True},
        "C4": {"x": 10, "y": 6, "c": False},
    }

    #quadcopter, sensors = create_coords(points)
    move_drone_to_sensor('C1')
    move_drone_to_sensor('C2')
    #print(f"Scene created with quadcopter {quadcopter} and {len(sensors)} sensors")
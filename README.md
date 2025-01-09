# Drone Simulation with Energy and Wind Effect

This project simulates the movement of a drone between predefined points while accounting for energy consumption and wind effects on specific moves.

## Features
- Simulates drone movement between various points.
- Calculates energy consumption for each move.
- Adds wind effects to certain moves, influencing the drone's performance.

## Setup Instructions
1. Place the file `sim_control_w_energy_wind_per_move_2.py` in your project directory.
2. Import the `DroneSimulation` class from the file as follows:
    ```python
    from sim_control_w_energy_wind_per_move_2 import DroneSimulation
    ```

## Usage
1. Define the points the drone will visit 

    Example:
    ```python
    points = {
        "B": {"x": 5, "y": 7},
        "C1": {"x": 1, "y": 5, "c": True},
        "C2": {"x": 2, "y": 1, "c": False},
        "C3": {"x": 6, "y": 2, "c": True},
        "C4": {"x": 10, "y": 6, "c": False},
    }
    ```

2. Specify the moves where wind effects will be present:
    ```python
    wind_moves = [4, 5]  # The 4th and 6th moves will have wind
    ```

3. Create an instance of the `DroneSimulation` class with the defined parameters:
    ```python
    simulation = DroneSimulation(points, wind_moves)
    ```

4. Use the additional function `get_energy()` to retrieve the energy details for the simulation:
    ```python
    energy_consumed = simulation.get_energy()
    print(f"Total energy consumed: {energy_consumed}")
    ```

## Example Code
```python
from sim_control_w_energy_wind_per_move_2 import DroneSimulation

# Define points
points = {
    "B": {"x": 5, "y": 7},
    "C1": {"x": 1, "y": 5, "c": True},
    "C2": {"x": 2, "y": 1, "c": False},
    "C3": {"x": 6, "y": 2, "c": True},
    "C4": {"x": 10, "y": 6, "c": False},
}

# Moves where wind is present
wind_moves = [4, 5]  # The 4th and 6th moves will have wind

# Initialize the simulation
simulation = DroneSimulation(points, wind_moves)

# Retrieve energy consumption
energy_consumed = simulation.get_energy()
print(f"Total energy consumed: {energy_consumed}")

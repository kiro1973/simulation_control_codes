from random import choice
from my_MCTS import MCTS, Node 
import math
import matplotlib.pyplot as plt
from config import *
from drone_coords_6 import DroneSimulation, EnergyWindow, DroneMonitor
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QObject, pyqtSignal
import threading
import time

# Configure matplotlib to use Qt5 backend
import matplotlib
matplotlib.use('Qt5Agg')

class PlotSignals(QObject):
    plot_signal = pyqtSignal(dict, list, bool)

class Node(Node):
    next_node_id = 0

    def __init__(self, sensor, parent, terminal):
        self.id = Node.next_node_id
        Node.next_node_id +=1
        self.sensor = sensor 
        self.parent = parent
        self.terminal = terminal
        self.accumulated_cost_mode_LO = 0 
        self.accumulated_cost_mode_HI = 0 
        self.compute_accumulated_cost()
        print(f"Node Created: ID={self.id}, Sensor={self.sensor}, Parent={self.parent.sensor if self.parent else None}, Terminal={self.terminal}")

    def compute_accumulated_cost(self):
        if (self.parent == None):
            self.accumulated_cost_mode_LO = 0 
            self.accumulated_cost_mode_HI = 0 
        else:
            dist_from_parent_to_current_node = calculate_distance(points[self.parent.sensor], points[self.sensor])
            print("Tif_dist", dist_from_parent_to_current_node)
            self.accumulated_cost_mode_LO = self.parent.accumulated_cost_mode_LO + coef_energy_no_wind*dist_from_parent_to_current_node
            
            if (points[self.sensor]["c"] == True):
                node = self
                worst_accumulated_cost = node.accumulated_cost_mode_HI
                while (node.parent != None and points[node.sensor]["c"]==False):
                    node = node.parent
                    if (node.accumulated_cost_mode_HI > worst_accumulated_cost):
                        worst_accumulated_cost = node.accumulated_cost_mode_HI

                self.accumulated_cost_mode_HI = worst_accumulated_cost + coef_energy_wind*dist_from_parent_to_current_node
            else:
                self.accumulated_cost_mode_HI = self.parent.accumulated_cost_mode_LO + coef_energy_wind*dist_from_parent_to_current_node

        print("accumulated_cost_mode_HI", self.id, " : ", self.sensor, " : ", self.accumulated_cost_mode_HI)
        print("accumulated_cost_mode_LO", self.id, " : ", self.sensor, " : ", self.accumulated_cost_mode_LO)

    def find_children(self):
        if self.terminal:
            return set()
        
        visited_sensors = set()
        node = self
        while (node.parent != None):
            visited_sensors.add(node.sensor)
            node = node.parent 

        unvisited_sensors = [
            sensor for sensor in points.keys() if sensor not in visited_sensors and sensor != "B"
        ]

        return {
            self.choose_next_sensor(sensor) for sensor in unvisited_sensors
        }
    
    def find_closer_child(self):
        print("find_closer_child")
        if self.terminal:
            return None
        visited_sensors = set()
        node = self
        while (node.parent != None):
            visited_sensors.add(node.sensor)
            node = node.parent
        print("visited_sensors", visited_sensors)

        unvisited_sensors = [
            sensor for sensor in points.keys() if sensor not in visited_sensors and sensor != "B"
        ]

        min_distance = float("inf")
        closest_sensor = None
        for un_sensor in unvisited_sensors:
            distance = calculate_distance(points[self.sensor], points[un_sensor])
            if distance < min_distance:
                min_distance = distance 
                closest_sensor = un_sensor
        return self.choose_next_sensor(closest_sensor)

    def is_terminal(self):
        return self.terminal

    def reward(self):
        if not self.terminal:
            raise RuntimeError(f"reward called on nonterminal self {self}")
        else:
            total_dist = calculate_accumulated_distance_drone(self)
            max_dist = calculate_distance_totale_max()
            
            reward = 0
            total_nber_sensors_ET_poids = (len(critical_sensors)*reward_critical_sensors + len(non_critical_sensors)*reward_non_critical_sensors)
            node = self
            while (node.parent != None):
                sensor = node.sensor
                if sensor in critical_sensors:
                    reward += reward_critical_sensors/total_nber_sensors_ET_poids
                else:
                    reward += reward_non_critical_sensors/total_nber_sensors_ET_poids
                node = node.parent 
            reward -= (1/total_nber_sensors_ET_poids)*(total_dist/max_dist)
            return reward
    
    def choose_next_sensor(self, sensor):
        dist_next_to_base = calculate_distance(points[sensor], points["B"])
        dist_to_next = calculate_distance(points[self.sensor], points[sensor])
        
        if (self.accumulated_cost_mode_HI + coef_energy_wind*(dist_to_next+dist_next_to_base) > init_energy or 
            self.accumulated_cost_mode_LO + coef_energy_no_wind*(dist_next_to_base+dist_next_to_base) > init_energy):      
            return Node(sensor="B", parent=self, terminal=True)
        else:
            return Node(sensor=sensor, parent=self, terminal=False)

def calculate_distance(capteur1, capteur2):
    x1, y1 = capteur1['x'], capteur1['y']
    x2, y2 = capteur2['x'], capteur2['y']
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_accumulated_distance_drone(last_node):
    accumulated_dist = 0 
    node = last_node
    while (node.parent != None):
        accumulated_dist += calculate_distance(points[node.sensor], points[node.parent.sensor])
        node = node.parent 
    return accumulated_dist

def calculate_distance_totale_max():
    remaining = points.copy()
    current = remaining.pop("B")
    total = 0
    while remaining:
        next_name, next_point = max(remaining.items(), key=lambda p: calculate_distance(current, p[1]))
        total += calculate_distance(current, next_point)
        current = remaining.pop(next_name)
    return total

def plot_capteurs_points(points, visited_sensors, is_HI_Mode):
    plt.clf()
    plt.gcf().set_size_inches(10, 8) 
    for name, coord in points.items():
        x, y = coord["x"], coord["y"]
        if name == "B":
            color = "black" 
        elif name in critical_sensors:
            color = "red"
        elif name in non_critical_sensors:
            color = "green"
        plt.scatter(x, y, color=color, label=name if name == "B" else "", s=100)
        offset_x = 0.3
        plt.text(x + offset_x, y, name, fontsize=10, ha='left', va='center')

    if visited_sensors:
        x_coords = [points["B"]["x"]]
        y_coords = [points["B"]["y"]]
        for sensor in visited_sensors:
            x_coords.append(points[sensor]["x"])
            y_coords.append(points[sensor]["y"])
        x_coords.append(points["B"]["x"])
        y_coords.append(points["B"]["y"])
        
        if (is_HI_Mode):
            plt.plot(x_coords, y_coords, color="orange", linestyle="--", marker="o", label="Trajet du drone")
        else:
            plt.plot(x_coords, y_coords, color="black", linestyle="--", marker="o", label="Trajet du drone")

    plt.title("Position des capteurs et de la base")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axhline(0, color="black", linewidth=0.5)
    plt.axvline(0, color="black", linewidth=0.5)
    plt.legend()
    plt.draw()
    plt.pause(0.1)

class DroneController:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.energy_window = EnergyWindow()
        self.simulation = None
        self.monitor = None
        self.algorithm_thread = None
        self.plot_signals = PlotSignals()
        self.plot_signals.plot_signal.connect(self.plot_in_main_thread)
        
    def initialize(self):
        self.simulation = DroneSimulation(points, wind_moves)
        self.simulation.energy_updated.connect(self.energy_window.update_energy)
        self.monitor = DroneMonitor(self.simulation, self.energy_window)
        self.monitor.start()
        
    def plot_in_main_thread(self, points_data, visited_sensors, is_HI_Mode):
        plot_capteurs_points(points_data, visited_sensors, is_HI_Mode)
        
    def run_algorithm(self, algorithm_function):
        self.algorithm_thread = threading.Thread(
            target=lambda: algorithm_function(self.simulation, self.plot_signals)
        )
        self.algorithm_thread.daemon = True
        self.algorithm_thread.start()
    
    def start(self):
        try:
            self.app.exec_()
        finally:
            if self.monitor:
                self.monitor.stop()
                self.monitor.join()

def do_drone_navigation(simulation, plot_signals):
    tree = MCTS()
    node = Node(sensor="B", parent=None, terminal=False)
    visited_sensors = []
    sensors_since_last_replanning = 0
    is_HI_Mode = False
    
    while True:
        if node.terminal:
            break
            
        if sensors_since_last_replanning >= 3 or node.parent == None:
            for _ in range(nber_of_rollout_iterations):
                tree.do_rollout(node)
            sensors_since_last_replanning = 0
            print("Do algo")

        node = tree.choose(node)
        visited_sensors.append(node.sensor)
        sensors_since_last_replanning += 1
        
        print(f"Capteurs visités : {visited_sensors}")
        
        if (is_HI_Mode):
            if(points[node.sensor]["c"] == True):
                simulation.move_drone_to_sensor(node.sensor)
        else:
            simulation.move_drone_to_sensor(node.sensor)

        real_energy_consumed = simulation.get_consumed_energy()
        dist = calculate_accumulated_distance_drone(node)
        print("energy_consumed_LO", node.accumulated_cost_mode_LO)
        print("energy_consumed_HI", node.accumulated_cost_mode_HI)
        print("accumulated_dist", dist)
        print("real_energy_consumed", real_energy_consumed)
       
        if (real_energy_consumed > node.accumulated_cost_mode_LO):
            is_HI_Mode = True
        print("is_HI_Mode", is_HI_Mode)
        
        energy_remaining = init_energy - real_energy_consumed
        print(f"Energy restante drône : {energy_remaining}")
        
        
        time.sleep(0.1)

    print(f"Nombre de capteurs visités : {len(visited_sensors)}")
    opti_total_dist = calculate_accumulated_distance_drone(node)
    print(f"Distance totale parcourue par le drône (opti) : {opti_total_dist}")
    print(f"Final Reward : ", node.reward())
    plot_signals.plot_signal.emit(points, visited_sensors, is_HI_Mode)

if __name__ == "__main__":
    controller = DroneController()
    controller.initialize()
    controller.run_algorithm(do_drone_navigation)
    controller.start()
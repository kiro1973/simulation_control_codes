from random import choice
from my_MCTS import MCTS, Node 
import math
import matplotlib.pyplot as plt
from config import *
from drone_coords_7 import DroneSimulation, EnergyWindow, DroneMonitor
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QObject, pyqtSignal
import threading
import time

# Configure matplotlib to use Qt5 backend
import matplotlib
matplotlib.use('Qt5Agg')

is_HI_Mode = False

class PlotSignals(QObject):
    plot_signal = pyqtSignal(dict, list, list, list)

class Node(Node):
    next_node_id = 0
    #Test2

    def __init__(self, sensor, parent, terminal):
        self.id = Node.next_node_id
        Node.next_node_id +=1
        self.sensor = sensor 
        self.parent = parent
        self.terminal = terminal
        self.accumulated_cost_mode_LO = 0 
        self.accumulated_cost_mode_HI = 0
        self.compute_accumulated_cost()
        #print(f"Node Created: ID={self.id}, Sensor={self.sensor}, Parent={self.parent.sensor if self.parent else None}, Terminal={self.terminal}")

    def compute_accumulated_cost(self):
        if (self.parent == None):
            self.accumulated_cost_mode_LO = 0 
            self.accumulated_cost_mode_HI = 0 
        else:
            """real_parent = self.parent 
            while (real_parent.deleted == True):
                real_parent = real_parent.parent
            """     
            #dist_from_parent_to_current_node = calculate_distance(points[self.parent.sensor], points[self.sensor])
            dist_from_parent_to_current_node = calculate_distance(points[self.parent.sensor], points[self.sensor])
            self.accumulated_cost_mode_LO = self.parent.accumulated_cost_mode_LO + coef_energy_no_wind_max*dist_from_parent_to_current_node
            
            if (points[self.sensor]["c"] == True):
                node = self.parent
                worst_accumulated_cost = node.accumulated_cost_mode_HI
                longest_dist_from_previous_to_curent_node = dist_from_parent_to_current_node
                while (node.parent != None and points[node.sensor]["c"]==False):
                    node = node.parent
                    if (node.accumulated_cost_mode_HI > worst_accumulated_cost):
                        worst_accumulated_cost = node.accumulated_cost_mode_HI

                    """dist_to_compare = calculate_distance(points[node.sensor], points[self.sensor])
                    if (dist_to_compare > longest_dist_from_previous_to_curent_node):
                        longest_dist_from_previous_to_curent_node = dist_to_compare 
                    """                  

                self.accumulated_cost_mode_HI = worst_accumulated_cost + coef_energy_wind_max*longest_dist_from_previous_to_curent_node
            else:
                self.accumulated_cost_mode_HI = self.parent.accumulated_cost_mode_LO + coef_energy_wind_max*dist_from_parent_to_current_node

        #print("accumulated_cost_mode_HI", self.id, " : ", self.sensor, " : ", self.accumulated_cost_mode_HI)
        #print("accumulated_cost_mode_LO", self.id, " : ", self.sensor, " : ", self.accumulated_cost_mode_LO)

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

        if (len(unvisited_sensors) != 0):
            return {
                    self.choose_next_sensor(sensor) for sensor in unvisited_sensors
            }
        else : #Gestion du cas où on a visité tous les capteurs 
            return {
                    self.choose_next_sensor("B") 
            }
    
    def find_closer_child(self):
        if self.terminal:
            return None
        visited_sensors = set()
        node = self
        while (node.parent != None):
            visited_sensors.add(node.sensor)
            node = node.parent

        unvisited_sensors = [
            sensor for sensor in points.keys() if sensor not in visited_sensors and sensor != "B"
        ]

        min_distance = float("inf")
        closest_sensor = "B"
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
            reward -= (reward_non_critical_sensors/total_nber_sensors_ET_poids)*(total_dist/max_dist)
            return reward
    
    def choose_next_sensor(self, sensor):
        #print(is_HI_Mode)
        #dist_next_to_base = calculate_distance(points[sensor], points["B"])
        #dist_to_next = calculate_distance(points[self.sensor], points[sensor])

        temp_node = Node(sensor = sensor, parent = self, terminal = False)
        temp_child = Node(sensor="B", parent=temp_node, terminal = True)

        #Gestion du cas où on a visité tous les capteurs 
        if (sensor == "B"): 
            return Node(sensor = sensor, parent = self, terminal = True)
        #elif ((self.accumulated_cost_mode_HI + coef_energy_wind*(dist_to_next+dist_next_to_base) >= init_energy and is_HI_Mode == True)  or (self.accumulated_cost_mode_LO + coef_energy_no_wind*(dist_to_next+dist_next_to_base) >= init_energy  and is_HI_Mode == False)):      
        
        #Dans le cas où il n'y a jamais de vent on parcourt la sequence entière ! 
        #Si on met la condition d'arrêt de la sequence avec HI, on optimise pas du tt l'énergy du drone... (il reste plein d'énergy à la fin...)
        #elif (self.accumulated_cost_mode_LO + coef_energy_no_wind*(dist_to_next+dist_next_to_base) >= init_energy ):  
        
        elif temp_child.accumulated_cost_mode_HI <= init_energy : 
            return temp_node
        else : 
            return Node(sensor = "B", parent = self, terminal = True)
        
        """elif (self.accumulated_cost_mode_HI + coef_energy_wind_max*(dist_to_next+dist_next_to_base) >= init_energy): 
        #elif (self.accumulated_cost_mode_HI + coef_energy_wind*(dist_to_next+dist_next_to_base) >= init_energy ): 
            return Node(sensor="B", parent=self, terminal=True)
        else:
            return Node(sensor=sensor, parent=self, terminal=False)
        """

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

def plot_capteurs_points(points, real_visited_sensors, deleted_sensors, is_HI_Mode_list):
    plt.clf()
    plt.gcf().set_size_inches(10, 8) 
    # Tracer les capteurs
    for name, coord in points.items():
        x, y = coord["x"], coord["y"]
        if name in deleted_sensors:  # Si le capteur est supprimé
            plt.scatter(x, y, color="gray", marker="x", label="Deleted Sensor" if "Deleted Sensor" not in plt.gca().get_legend_handles_labels()[1] else "", s=100)
        elif name == "B":  # Base
            plt.scatter(x, y, color="black", marker="o", label="Base" if "Base" not in plt.gca().get_legend_handles_labels()[1] else "", s=100)
        elif name in critical_sensors:  # Capteurs critiques
            plt.scatter(x, y, color="red", marker="o", label="Critical Sensor" if "Critical Sensor" not in plt.gca().get_legend_handles_labels()[1] else "", s=100)
        elif name in non_critical_sensors:  # Capteurs non-critiques
            plt.scatter(x, y, color="green", marker="o", label="Non-Critical Sensor" if "Non-Critical Sensor" not in plt.gca().get_legend_handles_labels()[1] else "", s=100)
        
        # Ajouter les étiquettes des capteurs
        offset_x = 0.3
        plt.text(x + offset_x, y, name, fontsize=10, ha='left', va='center')

    if real_visited_sensors:
        label_hi_mode_shown = False
        label_lo_mode_shown = False 
        for i in range(len(real_visited_sensors) - 1):  # Parcourt les segments
            sensor_start = real_visited_sensors[i]
            sensor_end = real_visited_sensors[i + 1]
            color = "orange" if is_HI_Mode_list[i] else "gray"
            if is_HI_Mode_list[i] and not label_hi_mode_shown:
                label = "Drone Motion in HI Mode"
                label_hi_mode_shown = True
            elif not is_HI_Mode_list[i] and not label_lo_mode_shown:
                label = "Drone Motion in LO Mode"
                label_lo_mode_shown = True
            else:
                label = None 
            # Récupérer les coordonnées de départ et d'arrivée pour le segment
            x_coords_segment = [points[sensor_start]["x"], points[sensor_end]["x"]]
            y_coords_segment = [points[sensor_start]["y"], points[sensor_end]["y"]]

            # Tracer le segment avec la couleur appropriée
            plt.plot(x_coords_segment, y_coords_segment, color=color, linestyle="--", marker="o", label=label if label else "")


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
        
    def plot_in_main_thread(self, points_data, visited_sensors, deleted_sensors, is_HI_Mode_list):
        plot_capteurs_points(points_data, visited_sensors, deleted_sensors, is_HI_Mode_list)
        
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
    global is_HI_Mode

    tree = MCTS()
    node = Node(sensor="B", parent=None, terminal=False)
    sensors_since_last_replanning = 0
    visited_sensors = []
    real_visited_sensors =[]
    deleted_sensors = []
    visited_sensors.append(node.sensor)
    real_visited_sensors.append(node.sensor)
    is_HI_Mode_list = []
    
    while True:
        if node.terminal:
            break
            
        if sensors_since_last_replanning >= 3 or node.parent == None:
            for _ in range(nber_of_rollout_iterations):
                tree.do_rollout(node)
            sensors_since_last_replanning = 0
            print("Do algo")




        #Move to next node 
        node = tree.choose(node)
        visited_sensors.append(node.sensor)
        sensors_since_last_replanning += 1

        """dist_next_to_base = calculate_distance(points[node.sensor], points["B"])
        dist_to_next = calculate_distance(points[node.parent.sensor], points[node.sensor])
        print("check node parent",node.parent.sensor)
        check = node.parent.accumulated_cost_mode_HI + coef_energy_wind*(dist_to_next+dist_next_to_base)
        print("check_stop_cond",check, node.sensor)
        """

        #Déplacement ou delete en fonction du mode du current node
        
        if (is_HI_Mode):
            if(points[node.sensor]["c"] == True):
                simulation.move_drone_to_sensor(node.sensor, is_HI_Mode, node.accumulated_cost_mode_HI, node.accumulated_cost_mode_LO)
                real_visited_sensors.append(node.sensor)
                is_HI_Mode_list.append(is_HI_Mode)
                #Mise à jour accumulated cost (cause deleted node) !! FAUX ???????????
                #node.compute_accumulated_cost() 

                #Mise à jour du mode
                real_energy_consumed = simulation.get_consumed_energy()
                if (real_energy_consumed > node.accumulated_cost_mode_LO):
                    is_HI_Mode = True
                else : 
                    is_HI_Mode = False


            else :
                deleted_sensors.append(node.sensor)
                #Modif online dans le cas node deleted ????
                """node.deleted = True
                node.accumulated_cost_mode_HI = node.parent.accumulated_cost_mode_HI
                node.accumulated_cost_mode_LO = node.parent.accumulated_cost_mode_LO
                """

        else:
            simulation.move_drone_to_sensor(node.sensor, is_HI_Mode, node.accumulated_cost_mode_HI, node.accumulated_cost_mode_LO)
            real_visited_sensors.append(node.sensor)
            is_HI_Mode_list.append(is_HI_Mode)
            #Mise à jour accumulated cost (cause deleted node) !! FAUX ???????????
            #node.compute_accumulated_cost() 
            
            #Mise à jour du mode
            real_energy_consumed = simulation.get_consumed_energy()
            if (real_energy_consumed > node.accumulated_cost_mode_LO):
                is_HI_Mode = True
            else : 
                is_HI_Mode = False

        
        time.sleep(0.1)

    #PRINT
    dist_to_go = calculate_distance(points[node.parent.sensor], points[node.sensor])
    print("dist_from_parent_to_current",dist_to_go)
    #dist = calculate_accumulated_distance_drone(node)
    print(f"Capteurs visités : {visited_sensors}")
    print(f"Capteurs visités (réel): {real_visited_sensors}")
    print(f"Deleted sensor :  {deleted_sensors}")
    print("energy_consumed_LO", node.accumulated_cost_mode_LO, node.sensor)
    print("energy_consumed_HI", node.accumulated_cost_mode_HI, node.sensor)
    #print("accumulated_dist", dist)
    print("real_energy_consumed", real_energy_consumed)
    print("is_HI_Mode", is_HI_Mode)


    

    print(f"Nombre de capteurs visités : {len(visited_sensors)}")
    opti_total_dist = calculate_accumulated_distance_drone(node)
    print(f"Distance totale parcourue par le drône (opti) : {opti_total_dist}")
    energy_remaining = init_energy - real_energy_consumed
    print(f"Energy restante drône : {energy_remaining}")
    print(f"Final Reward : ", node.reward())
    plot_signals.plot_signal.emit(points, real_visited_sensors, deleted_sensors, is_HI_Mode_list)

if __name__ == "__main__":
    controller = DroneController()
    controller.initialize()
    controller.run_algorithm(do_drone_navigation)
    controller.start()
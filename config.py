# =============================================
#           FICHIER DE CONFIGURATION
# =============================================

# =============================================
#       Déclaration des paramètres initiaux 
# =============================================

#Définition des emplacements des capteurs et de la base  
points = {
    "B": {"x": 5, "y": 7, "c": True}, #Correct de concidérer B comme critique ?
    "C1": {"x": 1, "y": 5, "c": True},
    "C2": {"x": 2, "y": 1, "c": False},
    "C3": {"x": 6, "y": 2, "c": True},
    "C4": {"x": 10, "y": 6, "c": False},
    "C5": {"x": 7, "y": 3, "c": True},
    "C6": {"x": 4, "y": 9, "c": False},
    "C7": {"x": 3, "y": 4, "c": True},
    "C8": {"x": 8, "y": 1, "c": False},
    "C9": {"x": 6, "y": 6, "c": True},
    "C10": {"x": 9, "y": 5, "c": False},
    "C11": {"x": 2, "y": 8, "c": True},
    "C12": {"x": 7, "y": 8, "c": False},
    "C13": {"x": 3, "y": 7, "c": True},
    "C14": {"x": 1, "y": 3, "c": False},
    "C15": {"x": 4, "y": 2, "c": True},
    "C16": {"x": 8, "y": 4, "c": False},
    "C17": {"x": 10, "y": 2, "c": False},
    "C18": {"x": 9, "y": 8, "c": False},
    "C19": {"x": 5, "y": 5, "c": False},
    "C20": {"x": 2, "y": 2, "c": False},
    }

critical_sensors = {key: value for key, value in points.items() if value.get("c") == True}
non_critical_sensors = {key: value for key, value in points.items() if value.get("c") == False}

init_energy = 120

coef_energy_no_wind_max = 2 # energy_consumed = coef_energy * dist
coef_energy_wind_max = 3

coef_energy_no_wind_real = 1.9
coef_energy_wind_real = 2.9

reward_critical_sensors = len(non_critical_sensors) +1
reward_non_critical_sensors = 1

replanning_freq = 3 #Décide les n prochains capteurs à visiter (refait le calcul de décision tous les n !)
nber_of_rollout_iterations = 500



#Pour moi infinie...
simulation_depth = 4 #Profondeur de simulation : on simule les 4 prochains capteurs afin de calculer la distance totale qui sera parcourue ! 


# Moves where wind is present
wind_moves = [3,4,5,6,7,8]  # The 4th and 5th moves will have wind
#wind_moves = []




from abc import ABC, abstractmethod
from collections import defaultdict
import math




class MCTS:
    "Monte Carlo tree searcher. First rollout the tree then choose a move."
    #Attributes at TREE level
    def __init__(self, exploration_weight=1.4):#1.2
        self.Q = defaultdict(int)  # total reward of each node
        self.N = defaultdict(int)  # total visit count for each node
        self.children = dict()  # children of each node
        self.exploration_weight = exploration_weight

    def choose(self, node):
        "Choose the best successor of node. (Choose a move in the game)"
        if node.is_terminal():
            raise RuntimeError(f"choose called on terminal node {node}")

        if node not in self.children:
            #return node.find_random_child()
            return node.find_closer_child()

        def score(n):
            if self.N[n] == 0:
                return float("-inf")  # avoid unseen moves
            return self.Q[n] / self.N[n]  # average reward

        return max(self.children[node], key=score)

    def do_rollout(self, node):
        "Make the tree one layer better. (Train for one iteration.)"
        "Do one iteration of the algo MCTS"
        path = self._select(node)
        leaf = path[-1]
        self._expand(leaf)
        reward = self._simulate(leaf)
        self._backpropagate(path, reward)

    def _select(self, node): #Convention : _method() doit être vue comme une méthode privée à la classe ! 
        "Find an unexplored descendent of `node`"
        path = [] #liste permettant de stocker les noeuds visités pendant la descente 
        while True:
            path.append(node) #Ajout du noeud courant à la liste 
            if node not in self.children or not self.children[node]:
                # node is either unexplored or terminal
                return path
            
            unexplored = self.children[node] - self.children.keys()
            if unexplored:
                n = unexplored.pop()
                path.append(n)
                return path
            node = self._uct_select(node)  # descend a layer deeper

    def _expand(self, node):
        "Update the `children` dict with the children of `node`"
        if node in self.children:
            return  # already expanded
        self.children[node] = node.find_children()
        #print(f"Node {node.sensor} expanded. Number of children: {len(self.children[node])}.") #ok B is well expanded ! Have 20 children 
        
       

    # def _simulate(self, node):
    #     "Returns the reward for a random simulation (to completion) of `node`"
    #     invert_reward = True
    #     while True:
    #         if node.is_terminal():
    #             reward = node.reward()
    #             return 1 - reward if invert_reward else reward
    #         node = node.find_random_child()
    #         invert_reward = not invert_reward

#Test all paths !!
        """    
        def _simulate(self, node): #Pour mon drône (but minimiser la distance, reward = -distance => maximiser le reward)
            
            if node.is_terminal():
                reward = node.reward()
                return reward
            #node = node.find_random_child() #Condition plus smart que random ? Distance la plus proche ? 

            all_children = node.find_children()
            best_reward = float('-inf')
            best_node = None
            
            for child in all_children:
                reward = self._simulate(child)  # Simuler chaque enfant
                if reward > best_reward:
                    best_reward = reward
                    best_node = child
            
            return best_reward

        """
    def _simulate(self, node): #Pour mon drône (but minimiser la distance, reward = -distance => maximiser le reward)
        while True :
            if node.is_terminal():
                reward = node.reward()
                #print("reward",reward)
                return reward
            #node = node.find_random_child() #Condition plus smart que random ? Distance la plus proche ? 
            node = node.find_closer_child()


    def _backpropagate(self, path, reward):
            "Send the reward back up to the ancestors of the leaf"
            for node in reversed(path):
                self.N[node] += 1
                self.Q[node] += reward

    def _uct_select(self, node): 
        "UTC = Upper Confidence bound for Tree"
        "Select a child of node, balancing exploration & exploitation"

        # All children of node should already be expanded:
        assert all(n in self.children for n in self.children[node])

        log_N_vertex = math.log(self.N[node])

        def uct(n):
            if (self.N[n] != 0) :
                "Upper confidence bound for trees"
                return self.Q[n] / self.N[n] + self.exploration_weight * math.sqrt(
                    log_N_vertex / self.N[n]
                )

        return max(self.children[node], key=uct)


class Node(ABC):
    """
    A representation of a single board state.
    MCTS works by constructing a tree of these Nodes.
    Could be e.g. a chess or checkers board state.
    """

    @abstractmethod
    def find_children(self):
        "All possible successors of this board state"
        return set()

    @abstractmethod
    def find_closer_child(self):
        "Random successor of this board state (for more efficient simulation)"
        return None

    @abstractmethod
    def is_terminal(self):
        "Returns True if the node has no children"
        return True

    @abstractmethod
    def reward(self):
        "Assumes `self` is terminal node. 1=win, 0=loss, .5=tie, etc"
        return 0


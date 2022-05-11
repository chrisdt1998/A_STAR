"""
This is the A* algorithm project which is visualized using Kivy. This project was designed and written by Christopher
du Toit with the intention to understand the A* algorithm and gain some basic understanding of Kivy.
"""
import math

from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout
from kivy.animation import Animation
from kivy.clock import Clock

import numpy as np


class AStarSearch:
    """
    A class used to represent the A* searching algorithm
    """
    def __init__(self, start, end, walls, grid_size, include_diag=False):
        """
        Initializing the A* search class
        :param start: x,y coordinate of starting position.
        :type start: list
        :param end: x,y coordinate of ending position.
        :type end: list
        :param walls: list of x,y coordinates of all wall blocks.
        :type walls: list
        :param grid_size: size n of n x n grid
        :type grid_size: int
        :param include_diag: Includes diagonal movement if True. Default is False.
        :type include_diag: bool
        """
        self.start = np.array(start)
        self.end = np.array(end)
        self.include_diag = include_diag

        self.max_size = grid_size - 1
        # Mask contains grid of -1 if node is a wall, 0 if it is open and 1 if it is closed (visited)
        mask = np.zeros((20, 20))
        for wall in walls:
            mask[wall[0]][wall[1]] = -1
        heuristics = np.zeros((20, 20))
        heuristics.fill(math.inf)
        costs = np.zeros((20, 20))
        costs.fill(math.inf)
        self.nodes = {'mask': mask, 'heuristics': heuristics, 'costs': costs}
        self.stored_nodes = []

    def compute_heuristic(self, node, from_start=True):
        """
        Method to compute heuristic function for the A* algorithm.
        :param node: x,y coordinate of the node.
        :type node: list
        :param from_start: True if computing heuristic while searching. False if backtracking to find optimal path.
        :type from_start: bool
        :return: Heuristic value of a particular position.
        :rtype: int
        """
        if from_start:
            goal_node = self.end
        else:
            goal_node = self.start

        if self.include_diag:
            len_diag = abs(node - goal_node).min()
            # len_diag = min(abs(node[0] - goal_node[0]), abs(node[1] - goal_node[1]))
            return (len_diag * 1.4) + np.sum(abs(node - goal_node)) - (len_diag * 2)
        else:
            return np.sum(abs(node - goal_node))

    def compute_cost(self, node, move):
        """
        Method to compute the cost from current node to the next node. Is just 1 if no diagonals included.
        :param node: x,y coordinate of the node.
        :type node: list
        :param move:
        :type move:
        :return:
        :rtype:
        """
        node_cost = self.nodes['costs'][node[0]][node[1]]
        if node_cost != math.inf:
            if np.sum(abs(node - move)) > 1 and self.include_diag:
                return node_cost + 1.4
            else:
                return node_cost + 1
        else:
            return 1



    def get_next_moves(self, node):
        """
        Method to compute all possible moves from a particular position
        :param node: x,y coordinate of position in question
        :type node: list
        :return: list of possible moves (does consider walls yet)
        :rtype: list
        """
        moves = []
        # Non-diags
        # Up
        if node[0] - 1 >= 0:
            moves.append([node[0] - 1, node[1]])
        # Up-right
        if self.include_diag and node[1] - 1 >= 0 and node[0] + 1 < self.max_size + 1:
            moves.append([node[0] + 1, node[1] - 1])
        # Right
        if node[1] + 1 < self.max_size + 1:
            moves.append([node[0], node[1] + 1])
        # Down-right
        if self.include_diag and node[0] + 1 < self.max_size + 1 and node[1] + 1 < self.max_size + 1:
            moves.append([node[0] + 1, node[1] + 1])
        # Down
        if node[0] + 1 < self.max_size + 1:
            moves.append([node[0] + 1, node[1]])
        # Down-left
        if self.include_diag and node[1] + 1 < self.max_size + 1 and node[0] - 1 >= 0:
            moves.append([node[0] - 1, node[1] + 1])
        # Left
        if node[1] - 1 >= 0:
            moves.append([node[0], node[1] - 1])
        # Up-left
        if self.include_diag and node[0] - 1 >= 0 and node[1] - 1 >= 0:
            moves.append([node[0] - 1, node[1] - 1])

        return np.array(moves)

    def search_neighbours(self, node, move):
        """
        This method checks if a move from a position is allowed and if so, it stores the cost and heuristic values of
        this move.
        :param node: x,y coordinate of the position.
        :type node:list
        :param move: x,y coordinate of the move.
        :type move: list
        """
        # Check move is available
        if self.nodes['mask'][move[0]][move[1]] == 0:
            # Check if we have already explored this move and update score if it is smaller now
            cost = self.compute_cost(node, move)
            heuristic = self.compute_heuristic(move)
            true_cost = self.nodes['costs'][move[0]][move[1]]
            if true_cost != math.inf:
                if true_cost > cost:
                    self.nodes['costs'][move[0]][move[1]] = cost
            else:
                # Add newly searched nodes
                self.stored_nodes.append(move.tolist())
                self.nodes['costs'][move[0]][move[1]] = cost
                self.nodes['heuristics'][move[0]][move[1]] = heuristic

    def get_position(self):
        """
        This method computes the next node to visit based on the A* algorithm, i.e. it chooses the move with the
        lowest cost.
        :return: x,y coordinate of chosen position
        :rtype: list
        """
        scores = self.nodes['heuristics'] + self.nodes['costs']
        scores_filtered = scores[self.nodes['mask'] == 0]

        index = np.argwhere(scores == scores_filtered.min())

        # If there are multiple options then explore the one with the smallest heuristic score.
        score = math.inf
        for i in index:
            if self.nodes['mask'][i[0]][i[1]] == 0:
                if self.nodes['heuristics'][i[0]][i[1]] < score:
                    node = i
                    score = self.nodes['heuristics'][i[0]][i[1]]
        return node

    def backtrack(self):
        shortest_path = [self.end.tolist()]
        node = self.end.tolist()
        counter = 0
        while node != self.start.tolist():
            neighbouring_nodes = self.get_next_moves(node).tolist()
            total_cost = 1000
            for n in neighbouring_nodes:
                if self.nodes['costs'][n[0]][n[1]] != math.inf:
                    n_cost = self.nodes['costs'][n[0]][n[1]]
                    if n_cost < total_cost:
                        chosen_n = n
                        total_cost = n_cost
            node = chosen_n
            shortest_path.append(chosen_n)
            counter += 1
            if counter > 50:
                break
        return shortest_path


    def search(self):
        node = self.start
        self.nodes['mask'][node[0]][node[1]] = 1
        self.nodes['heuristics'][node[0]][node[1]] = self.compute_heuristic(node)
        self.nodes['costs'][node[0]][node[1]] = 0
        self.stored_nodes.append(node.tolist())
        counter = 0
        while not np.array_equal(node, self.end):
            moves = self.get_next_moves(node)
            for move in moves:
                self.search_neighbours(node, move)

            node = self.get_position()
            self.nodes['mask'][node[0]][node[1]] = 1
            self.stored_nodes.append(node.tolist())

            counter += 1
            if counter > self.max_size ** 2:
                print("Could not find path")
                break

        print(self.nodes['mask'])
        print(self.nodes['heuristics'])
        print(self.nodes['costs'])
        print(self.nodes['costs'] + self.nodes['heuristics'])
        print(self.stored_nodes)
        print("Search finished")



class CreateGrid(GridLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.padding = 10
        self.cols = 20
        self.counter = 0
        self.iter = 0
        self.start = []
        self.end = []
        self.walls = []
        self.speed = 60
        self.num_walls = 15
        self.grid_size = 20
        self.visualized_nodes = []
        self.all_nodes = []
        self.shortest_path = []
        self.b = []
        for i in range(400):
            self.b.append(Button())
            self.b[i].bind(on_press=self.button_clicked)
            self.b[i].id_num = i
            self.add_widget(self.b[i])

    def button_clicked(self, button):
        if self.counter == 0:
            button.background_normal = ''
            anim = Animation(background_color = (0, 1, 0, .85))
            anim.start(button)
            self.start = [int(button.id_num / self.grid_size), int(button.id_num % self.grid_size)]
            print(f"Starting position {self.start}")
        elif self.counter == 1:
            button.background_normal = ''
            anim = Animation(background_color = (1, .3, .4, .85))
            anim.start(button)
            self.end = [int(button.id_num / self.grid_size), int(button.id_num % self.grid_size)]
            print(f"Ending position {self.end}")
        elif 1 < self.counter <= 2 + self.num_walls:
            if self.counter == 2 + self.num_walls:
                print("Start running")
                astar = AStarSearch(self.start, self.end, self.walls, self.grid_size, False)
                astar.search()
                self.shortest_path = astar.backtrack()
                print(len(self.shortest_path))
                self.all_nodes = astar.stored_nodes
                Clock.schedule_interval(self.animate, 1 / self.speed)
                print("Visualization finished")
            else:
                anim = Animation(background_color=(0, 0, 0, .85))
                anim.start(button)
                self.walls.append([int(button.id_num / self.grid_size), int(button.id_num % self.grid_size)])

        self.counter += 1

    def animate(self, dt):
        if self.iter < len(self.all_nodes):
            index = (self.all_nodes[self.iter][0] * self.grid_size) + self.all_nodes[self.iter][1]
            self.b[index].background_normal = ''
            if self.all_nodes[self.iter] not in self.visualized_nodes:
                if self.all_nodes[self.iter] != self.end and self.all_nodes[self.iter] != self.start:
                    anim = Animation(background_color=(0, 0, 1, .85))
                    anim.start(self.b[index])
                    self.visualized_nodes.append(self.all_nodes[self.iter])
            else:
                anim = Animation(background_color=(0.5, 0, 0.5, .85))
                anim.start(self.b[index])
        elif self.iter >= len(self.all_nodes) and self.iter < len(self.all_nodes) + len(self.shortest_path):
            i = self.iter - len(self.all_nodes)
            index = (self.shortest_path[i][0] * self.grid_size) + self.shortest_path[i][1]
            if self.shortest_path[i] != self.end and self.shortest_path[i] != self.start:
                self.b[index].background_normal = ''
                anim = Animation(background_color=(1, 215/255, 0, 1))
                anim.start(self.b[index])
        else:
            return False
        self.iter += 1


class AStarApp(App):
    def build(self):
        return CreateGrid()







num_clicks = 0
AStarApp().run()

# AStarApp().animate(visited_nodes, 10)


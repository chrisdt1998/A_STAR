import numpy as np
import math


class AStarSearch:
    """
    A class used to represent the A* searching algorithm
    """
    def __init__(self, start, end, walls, num_rows, num_cols, include_diag=False):
        """
        Initializing the A* search class
        :param start: x,y coordinate of starting node.
        :type start: list
        :param end: x,y coordinate of ending node.
        :type end: list
        :param walls: list of x,y coordinates of all wall nodes.
        :type walls: list
        :param grid_size: size n of n x n grid
        :type grid_size: int
        :param include_diag: Includes diagonal movement if True. Default is False.
        :type include_diag: bool
        """
        self.start = np.array(start)
        self.end = np.array(end)
        self.include_diag = include_diag
        self.failed = False

        self.num_rows = num_rows
        self.num_cols = num_cols
        # Mask contains grid of -1 if node is a wall, 0 if it is open and 1 if it is closed (visited)
        mask = np.zeros((num_rows, num_cols))
        for wall in walls:
            mask[wall[0]][wall[1]] = -1
        heuristics = np.zeros((num_rows, num_cols))
        heuristics.fill(math.inf)
        costs = np.zeros((num_rows, num_cols))
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
        :return: Heuristic value of a particular node.
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
        :param move: x,y coordinate of move, used for checking if a move is diagonal.
        :type move: list
        :return: cost of move
        :rtype: int
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
        Method to compute all possible moves from a particular node in clockwise order. Only considers diagonal moves
        if self.include_diag is True.
        :param node: x,y coordinate of node.
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
        if self.include_diag and node[1] - 1 >= 0 and node[0] + 1 < self.num_rows:
            moves.append([node[0] + 1, node[1] - 1])
        # Right
        if node[1] + 1 < self.num_cols:
            moves.append([node[0], node[1] + 1])
        # Down-right
        if self.include_diag and node[0] + 1 < self.num_rows and node[1] + 1 < self.num_cols:
            moves.append([node[0] + 1, node[1] + 1])
        # Down
        if node[0] + 1 < self.num_rows:
            moves.append([node[0] + 1, node[1]])
        # Down-left
        if self.include_diag and node[1] + 1 < self.num_cols and node[0] - 1 >= 0:
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
        This method checks if a move from a node is allowed and if so, it stores the cost and heuristic values of
        this move.
        :param node: x,y coordinate of the node.
        :type node: list
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
        This method computes the next node to visit based on the A* algorithm, i.e. it chooses the move which minimizes
        g(n) + h(n) where g(n) is the cost to the node n and h(n) is the heuristic of node n.
        :return: x,y coordinate of chosen node
        :rtype: list
        """
        scores = self.nodes['heuristics'] + self.nodes['costs']
        scores_filtered = scores[self.nodes['mask'] == 0]

        index = np.argwhere(scores == scores_filtered.min())

        # If there are multiple options then explore the one with the smallest heuristic score.
        node = None
        score = math.inf
        for i in index:
            if self.nodes['mask'][i[0]][i[1]] == 0:
                if self.nodes['heuristics'][i[0]][i[1]] < score:
                    node = i
                    score = self.nodes['heuristics'][i[0]][i[1]]
        return node

    def backtrack(self):
        """
        This method computes the shortest path, once the search is finished, by backtracking.
        :return: The shortest path from start to finish.
        :rtype: list
        """
        shortest_path = [self.end.tolist()]
        node = self.end.tolist()
        counter = 0
        while node != self.start.tolist():
            neighbouring_nodes = self.get_next_moves(node).tolist()
            total_cost = 1000
            for n in neighbouring_nodes:
                if self.nodes['costs'][n[0]][n[1]] != math.inf and self.nodes['mask'][n[0]][n[1]] == 1:
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
        """
        Call this method to start the A* searching process.
        """
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
            if node is None:
                print("Could not find path")
                self.failed = True
                break
            self.nodes['mask'][node[0]][node[1]] = 1
            self.stored_nodes.append(node.tolist())

            counter += 1
            if counter > self.num_rows ** 2:
                print("Could not find path")
                self.failed = True
                break
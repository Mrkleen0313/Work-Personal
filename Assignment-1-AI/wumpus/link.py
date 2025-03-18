# link.py
#
# The code that defines the behaviour of Link.
#
# You should be able to write the code for a simple solution to the
# game version of the Wumpus World here, getting information about the
# game state from self.gameWorld, and using makeMove() to generate the
# next move.
#
# Written by: Simon Parsons
# Last Modified: 25/08/20

import random
import math
from utils import Directions, Pose


class Link():
    def __init__(self, dungeon):
        # Make a copy of the world an attribute, so that Link can query the state of the world
        self.gameWorld = dungeon

        # What moves are possible.
        self.moves = [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]

        # frontier
        self.path = []

    def makeMove(self):
        # calculate the distances between your position and each gold position, use the smallest value available
        def calculate_distance(pos1, pos2):
            return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

        # Get the locations.
        allGold = self.gameWorld.getGoldLocation()
        myPosition = self.gameWorld.getLinkLocation()
        allPits = self.gameWorld.getPitsLocation()

        if not self.path:
            if len(allGold) > 0:
                nextGold = min(allGold, key=lambda gold: calculate_distance(myPosition, gold))
                allWumpus = self.gameWorld.getWumpusLocation()
                self.path = self.a_star(myPosition, nextGold, allWumpus)

                if not self.path:
                    return random.choice(self.moves)
            else:
                return random.choice(self.moves)

        next_move = self.path.pop(0)

        return next_move

    def a_star(self, start, goal, allWumpus):

        # Initialize the open and closed sets
        open_set = {} #store nodes to be evaluated
        closed_set = set() #closed set keeps track of already evaluated nodes

        #Create the start node with initial values (position, scores, parent, direction).
        start_node = {
            'pose': start, #This assigns the starting position to the node
            'g_score': 0, #represents the cost from the start to the current node
            'h_score': self.heuristic(start, goal), #the estimated cost from the current node to the goal
            'f_score': self.heuristic(start, goal), #the sum of g_score and h_score.  / same value as h_score because g_score is 0 for the start node
            'parent': None, #set to None for the start node, as it has no predecessor
            'direction': None, #represents the direction taken to reach this node
            'proximity_penalty': 0 #penalty heuristic
        }

        # Add start node to open set
        node_id = f"{start.x},{start.y}" #The node_id is a unique identifier for the node, created using its x and y coordinates.
        #f-string, formats the coordinates into a string in the format "x,y"
        open_set[node_id] = start_node

        # Main A* loop
        while open_set:
            # Find the node with the lowest f_score
            current_id = min(open_set, key=lambda id: open_set[id]['f_score'])
            current = open_set[current_id]

            # If we reached the goal, reconstruct the path
            if current['pose'].x == goal.x and current['pose'].y == goal.y:
                return self.reconstruct_path(current)

            # Move current node from open to closed set
            del open_set[current_id]
            closed_set.add(current_id)

            # Generate possible moves
            for direction in self.moves:
                # Get the new position after this move
                neighbor_pose = self.get_neighbor_position(current['pose'], direction)
                neighbor_id = f"{neighbor_pose.x},{neighbor_pose.y}"

                # Skip if this position is already in the closed set
                if neighbor_id in closed_set:
                    continue

                # Skip if this would move outside the world boundaries
                if not self.is_valid_position(neighbor_pose):
                    continue

                # Calculate g_score for this neighbor
                tentative_g_score = current['g_score'] + 1

                # Create the neighbor node
                neighbor_node = {
                    'pose': neighbor_pose,
                    'g_score': tentative_g_score,
                    'h_score': self.heuristic(neighbor_pose, goal),
                    'f_score': tentative_g_score + self.heuristic(neighbor_pose, goal) + 1 * self.calculateTotalPenalty(neighbor_pose, allWumpus),
                    'parent': current,
                    'direction': direction,
                }

                # If this node is not in the open set or has a better score
                if neighbor_id not in open_set or tentative_g_score < open_set[neighbor_id]['g_score']:
                    open_set[neighbor_id] = neighbor_node

        # If we've exhausted all possibilities and didn't find a path
        return []

    def reconstruct_path(self, current):
        #Returns a list of Directions to follow.
        path = []
        while current['parent'] is not None:
            path.append(current['direction'])
            current = current['parent']

        # Reverse the path to get start->goal order
        path.reverse()
        return path

    def heuristic(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def penaltyHeuristic(self, a, b):
        max_penalty = 55  # Adjust this value as needed
        terror = abs(a.x - b.x) + abs(a.y - b.y)  # Manhattan distance

        if terror == 0:
            return max_penalty  # Maximum penalty when overlapping
        else:
            return max_penalty / terror  # Higher penalty for closer distances

    def calculateTotalPenalty(self, pose, allWumpus):
        total_penalty = 0
        for wumpus in allWumpus:
            total_penalty += self.penaltyHeuristic(pose, wumpus)
        return total_penalty

    def get_neighbor_position(self, pose, direction):
        new_pose = Pose()
        new_pose.x = pose.x
        new_pose.y = pose.y

        if direction == Directions.NORTH:
            new_pose.y = pose.y + 1
        elif direction == Directions.SOUTH:
            new_pose.y = pose.y - 1
        elif direction == Directions.EAST:
            new_pose.x = pose.x + 1
        elif direction == Directions.WEST:
            new_pose.x = pose.x - 1

        return new_pose

    #Checks if the given position is within the world boundaries.
    def is_valid_position(self, pose):
        return (0 <= pose.x <= self.gameWorld.maxX) and (0 <= pose.y <= self.gameWorld.maxY)
# puzzleWorld.py
#
# A file that represents a puzzle version of the Wumpus World, keeping
# track of the position of the Wumpus and Link.
#
# Written by: Simon Parsons
# Last Modified: 17/12/24
import math
import random
import config
import utils
import copy
from world import World
from utils import Pose
from utils import Directions
from utils import State

class PuzzleWorld(World):

    def __init__(self):

        # Import boundaries of the world. because we index from 0,
        # these are one less than the number of rows and columns.
        self.maxX = config.worldLength - 1
        self.maxY = config.worldBreadth - 1

        # Keep a list of locations that have been used.
        self.locationList = []

        # Wumpus
        self.wLoc = []
        for i in range(config.numberOfWumpus):
            newLoc = utils.pickUniquePose(self.maxX, self.maxY, self.locationList)
            self.wLoc.append(newLoc)
            self.locationList.append(newLoc)

        # Link
        newLoc = utils.pickUniquePose(self.maxX, self.maxY, self.locationList)
        self.lLoc = newLoc
        self.locationList.append(newLoc)

        # Other elements that we don't use
        self.pLoc = []
        self.gLoc = []
        
        # Game state
        self.status = utils.State.PLAY

        # What moves are possible.
        self.moves = [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]

        # A plan
        self.plan = [[Directions.NORTH, 0, 0], [0, Directions.NORTH, 0], [0, 0, Directions.NORTH]]

    #
    # Methods
    #
    # These are the functions that are used to update and report on
    # puzzle information.
    def isSolved(self, goal):
        if utils.sameAs(self, goal):
            self.status = utils.State.WON 
            print("Puzzle Over!")
            return True
        else:
            return False

    # This is where you should start writing your solution to the
    # puzle problem.
    def makeAMove(self, goal):

        # If we've already solved the puzzle, no need to make more moves
        if self.isSolved(goal):
            return [0] * (len(self.wLoc) + 1)

        # Initialize paths for Wumpus or link if not already
        if not hasattr(self, 'wumpus_paths'):
            self.wumpus_paths = [[] for _ in range(len(self.wLoc))]
        if not hasattr(self, 'link_path'):
            self.link_path = []

        # Get the start and goal positions for all entitys
        startWumpus = self.getWumpusLocation()
        endWumpus = goal.getWumpusLocation()
        startLink = self.getLinkLocation()
        endLink = goal.getLinkLocation()

        # Create a move vector that will hold directions for Link and each Wumpus
        move = [0] * (len(startWumpus) + 1)  # +1 for Link

        # Update paths for each Wumpus if needed
        for i in range(len(startWumpus)):
            if not self.wumpus_paths[i]:
                # Calculate path using A* just like Link does
                self.wumpus_paths[i] = self.a_star(startWumpus[i], endWumpus[i], startWumpus)

            # Move each Wumpus if it has a path
            if self.wumpus_paths[i]:
                next_dir = self.wumpus_paths[i].pop(0)
                move[i + 1] = next_dir

                # Update the Wumpus's location based on the move
                current_wumpus_location = self.wLoc[i]
                new_wumpus_location = self.get_neighbor_position(current_wumpus_location, next_dir)

                # Check if the new location is valid
                if self.is_valid_position(new_wumpus_location):
                    self.wLoc[i] = new_wumpus_location

        # Calculate Link's path if it doesn't exist or is empty
        if not self.link_path:
            self.link_path = self.a_star(startLink, endLink, self.getWumpusLocation())

        # Move Link if there's a path
        if self.link_path:
            next_dir = self.link_path.pop(0)
            move[0] = next_dir

            new_link_location = self.get_neighbor_position(self.lLoc, next_dir)

            # Check if the new location is valid
            if self.is_valid_position(new_link_location):
                self.lLoc = new_link_location

        return move

    def a_star(self, start, goal, all_wumpus):
        # Initialize the open and closed sets
        open_set = {}  # store nodes to be evaluated
        closed_set = set()  # closed set keeps track of already evaluated nodes

        # Create the start node with initial values
        start_node = {
            'pose': start,
            'g_score': 0,
            'h_score': self.heuristic(start, goal),
            'f_score': self.heuristic(start, goal),
            'parent': None,
            'direction': None,
        }

        # Add start node to open set
        node_id = f"{start.x},{start.y}"
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

                # Skip if another Wumpus is at this position (except the goal)
                wumpus_collision = False
                for w in all_wumpus:
                    if (w.x == neighbor_pose.x and w.y == neighbor_pose.y and
                            (w.x != goal.x or w.y != goal.y)):
                        wumpus_collision = True
                        break

                # Calculate g_score for this neighbor
                tentative_g_score = current['g_score'] + 1

                # Create the neighbor node
                neighbor_node = {
                    'pose': neighbor_pose,
                    'g_score': tentative_g_score,
                    'h_score': self.heuristic(neighbor_pose, goal),
                    'f_score': tentative_g_score + self.heuristic(neighbor_pose, goal),
                    'parent': current,
                    'direction': direction,
                }

                # If this node is not in the open set or has a better score
                if neighbor_id not in open_set or tentative_g_score < open_set[neighbor_id]['g_score']:
                    open_set[neighbor_id] = neighbor_node

        # If we've exhausted all possibilities and didn't find a path
        return []

    def heuristic(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def reconstruct_path(self, current):
        # Returns a list of Directions to follow.
        path = []
        while current['parent'] is not None:
            path.append(current['direction'])
            current = current['parent']

        # Reverse the path to get start->goal order
        path.reverse()
        return path

    def get_neighbor_position(self, pose, direction):
        new_pose = utils.Pose()  # Make sure to use the correct import for Pose
        new_pose.x = pose.x
        new_pose.y = pose.y

        if direction == utils.Directions.NORTH:
            new_pose.y = pose.y + 1
        elif direction == utils.Directions.SOUTH:
            new_pose.y = pose.y - 1
        elif direction == utils.Directions.EAST:
            new_pose.x = pose.x + 1
        elif direction == utils.Directions.WEST:
            new_pose.x = pose.x - 1

        return new_pose

    def is_valid_position(self, pose):
        return (0 <= pose.x <= self.maxX) and (0 <= pose.y <= self.maxY)
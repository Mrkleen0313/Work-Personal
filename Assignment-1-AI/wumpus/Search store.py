def greedy_search(self, start, goal, all_wumpus):
    #
    #GREEDY SEARCH
    # Initialize the open and closed sets
    open_set = {}  # store nodes to be evaluated
    closed_set = set()  # closed set keeps track of already evaluated nodes

    # Create the start node with initial values
    start_node = {
        'pose': start,
        'h_score': self.heuristic(start, goal),
        'parent': None,
        'direction': None,
    }

    # Add start node to open set
    node_id = f"{start.x},{start.y}"
    open_set[node_id] = start_node

    # Main greedy search loop
    while open_set:
        # Find the node with the lowest h_score (closest to goal)
        current_id = min(open_set, key=lambda id: open_set[id]['h_score'])
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
            if wumpus_collision:
                continue

            # Create the neighbor node
            neighbor_node = {
                'pose': neighbor_pose,
                'h_score': self.heuristic(neighbor_pose, goal),
                'parent': current,
                'direction': direction,
            }

            # If this node is not in the open set or has a better score
            if neighbor_id not in open_set or neighbor_node['h_score'] < open_set[neighbor_id]['h_score']:
                open_set[neighbor_id] = neighbor_node

    # If we've exhausted all possibilities and didn't find a path
    return []

##UCS
    def uniform_cost_search(self, start, goal, allWumpus, allPits):
        open_set = {}
        closed_set = set()

        start_node = {
            'pose': start,
            'score': 0,
            'parent': None,
            'direction': None,
            'proximity_penalty': 0
        }

        node_id = f"{start.x},{start.y}"
        open_set[node_id] = start_node

        while open_set:
            current_id = min(open_set, key=lambda id: open_set[id]['score'])
            current = open_set[current_id]

            if current['pose'].x == goal.x and current['pose'].y == goal.y:
                return self.reconstruct_path(current)

            del open_set[current_id]
            closed_set.add(current_id)

            for direction in self.moves:
                neighbor_pose = self.get_neighbor_position(current['pose'], direction)
                neighbor_id = f"{neighbor_pose.x},{neighbor_pose.y}"

                if neighbor_id in closed_set:
                    continue

                if not self.is_valid_position(neighbor_pose):
                    continue

                if any(pit.x == neighbor_pose.x and pit.y == neighbor_pose.y for pit in allPits):
                    continue

                if any(wumpus.x == neighbor_pose.x and wumpus.y == neighbor_pose.y for wumpus in allWumpus):
                    continue

                new_score = current['score'] + 1 + self.calculateTotalPenalty(neighbor_pose, allWumpus)

                neighbor_node = {
                    'pose': neighbor_pose,
                    'score': new_score,
                    'parent': current,
                    'direction': direction,
                }

                if neighbor_id not in open_set or new_score < open_set[neighbor_id]['score']:
                    open_set[neighbor_id] = neighbor_node

        return []

    # ... (keep the other helper methods as they are)

    def makeMove(self):
        allWumpus = self.gameWorld.getWumpusLocation()
        allGold = self.gameWorld.getGoldLocation()
        myPosition = self.gameWorld.getLinkLocation()
        allPits = self.gameWorld.getPitsLocation()

        if len(allGold) > 0:
            nextGold = min(allGold, key=lambda gold: self.calculate_distance(myPosition, gold))
            self.path = self.uniform_cost_search(myPosition, nextGold, allWumpus, allPits)

            if not self.path:
                return random.choice(self.moves)
        else:
            return random.choice(self.moves)

        next_move = self.path.pop(0)
        return next_move

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

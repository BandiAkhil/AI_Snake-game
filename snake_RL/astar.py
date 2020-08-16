from __future__ import print_function
from move import Direction


class AStarGraph(object):
    # Define a class board like grid with two barriers

    def __init__(self, barriers, boardW, boardH, direction, body_parts):
        self.barriers = set(barriers)
        self.boardW = boardW
        self.boardH = boardH
        self.direction = direction
        self.body_parts = tuple(body_parts)
        self.depth = 0

    def heuristic(self, start, goal):
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        return dx + dy

    def get_vertex_neighbours(self, pos):
        n = []
        moves = [((1, 0), Direction.EAST), ((-1, 0), Direction.WEST), ((0, 1), Direction.SOUTH), ((0, -1), Direction.NORTH)]
        valid_moves = [move for move in moves if move[0] in pos[1].get_xy_moves()]
        for dxy, _direction in valid_moves:
            x2 = pos[0][0] + dxy[0]
            y2 = pos[0][1] + dxy[1]
            if x2 not in range(self.boardW) or y2 not in range(self.boardH):
                continue
            new_body = pos[2]
            if len(pos[2]) > 0:
                new_body = (pos[0], *pos[2][:-1])
            el = ((x2, y2), _direction, new_body)
            n.append(el)
        self.depth += 1
        return n

    def move_cost(self, a, b):
        if b[0] in self.barriers or b[0] in a[2]:
            return 10000  # Extremely high cost to enter barrier squares or snake body
        return 1  # Normal movement cost


def AStarSearch(start, end, graph, check_end=True):
    G = {}  # Actual movement cost to each position from the start position
    F = {}  # Estimated movement cost of start to end going via this position

    # Initialize starting values
    G[start[:-1]] = 0
    F[start[:-1]] = graph.heuristic(start[0], end)

    closedVertices = set()
    openVertices = {start[:-1]: start}
    cameFrom = {}

    while len(openVertices) > 0:
        # Get the vertex in the open list with the lowest F score
        current = None
        currentFscore = None
        for pos in openVertices.values():
            if current is None or F[pos[:-1]] < currentFscore:
                currentFscore = F[pos[:-1]]
                current = pos

        # Check if we have reached the goal
        if current[0] == end:
            # Retrace our route backward
            curr = current[:-1]
            path = [curr]

            while curr in cameFrom:
                curr = cameFrom[curr]
                path.append(curr)
            path.reverse()

            cost = [F[(end, d)] for d in [Direction.SOUTH, Direction.WEST, Direction.NORTH, Direction.EAST] if (end, d) in F][0]

            # If snake is short, it will never go to a dead end
            if len(current[2]) < 2 or not check_end:
                # Done! Just eat the food!
                return path, cost, True

            # Snake is too long. It can now go to a dead end after eating a food.
            else:
                # calculate what the body is going to be after following the resulting path
                body = start[2]
                if len(body) > 0:
                    for p in path:
                        body = (p[0], *body[:-1])

                # Calculate final state after eating the food
                end_pos = (*path[-1], body)

                # Find a route from food to be eaten to future tail.
                result, tail_cost, eaten = AStarSearch(end_pos, body[-1], graph, False)

                # Snake can follow it's tail after eating the food.
                if tail_cost < 10000:
                    # Eat the food
                    return path, cost, True

                # Snake can't follow the tail after eating the food
                else:
                    print("Cost of following tail after eating too large!")
                    start_tail = start[2][-1]
                    # follow tail now instead of eating the food
                    result, cost, eaten = AStarSearch(start, start_tail, graph, check_end=False)
                    return result, cost, False

        # Mark the current vertex as closed
        del openVertices[current[:-1]]
        closedVertices.add(current[:-1])

        # print("Expanding " + str(current))
        # Update scores for vertices near the current position
        for neighbour in graph.get_vertex_neighbours(current):
            if neighbour[:-1] in closedVertices:
                continue  # We have already processed this node exhaustively
            candidateG = G[current[:-1]] + graph.move_cost(current, neighbour)

            if neighbour[:-1] not in openVertices:
                openVertices[neighbour[:-1]] = neighbour  # Discovered a new vertex
            elif candidateG >= G[neighbour[:-1]]:
                continue  # This G score is worse than previously found

            # Adopt this G score
            cameFrom[neighbour[:-1]] = current[:-1]
            G[neighbour[:-1]] = candidateG
            H = graph.heuristic(neighbour[0], end)
            F[neighbour[:-1]] = candidateG + H

    raise RuntimeError("A* failed to find a solution")
"""
ROS-independent helpers for Theta* path planning.

Cells use (x, y); the grid is indexed as grid[y][x].
True marks an unavailable cell center.
A supplied segment checker validates complete connections.
Without one, the original grid line-of-sight policy is used.
"""

import heapq
import math

Cell = tuple[int, int]


def is_free(grid, cell: Cell) -> bool:
    """Return whether a cell is inside the grid and unblocked."""
    x, y = cell
    height = len(grid)
    width = len(grid[0]) if height else 0

    if not (0 <= x < width and 0 <= y < height):
        return False

    return not bool(grid[y][x])


def distance(a: Cell, b: Cell) -> float:
    """Return Euclidean distance in grid-cell units."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def line_of_sight(grid, start: Cell, end: Cell) -> bool:
    """Check a segment between cell centers without cutting corners."""
    if not is_free(grid, start) or not is_free(grid, end):
        return False

    x, y = start
    end_x, end_y = end

    dx = abs(end_x - x)
    dy = abs(end_y - y)
    step_x = (end_x > x) - (end_x < x)
    step_y = (end_y > y) - (end_y < y)

    crossed_x = 0
    crossed_y = 0

    while crossed_x < dx or crossed_y < dy:
        # Compare the next vertical and horizontal boundary crossings.
        # Integer arithmetic avoids floating-point tie errors.
        next_x = (2 * crossed_x + 1) * dy
        next_y = (2 * crossed_y + 1) * dx

        if next_x == next_y:
            # At a corner, both adjacent side cells must be free.
            if not is_free(grid, (x + step_x, y)):
                return False
            if not is_free(grid, (x, y + step_y)):
                return False

            x += step_x
            y += step_y
            crossed_x += 1
            crossed_y += 1
        elif next_x < next_y:
            x += step_x
            crossed_x += 1
        else:
            y += step_y
            crossed_y += 1

        if not is_free(grid, (x, y)):
            return False

    return True


def theta_star(grid, start: Cell, goal: Cell, segment_is_free=None) -> list[Cell]:
    """
    Return cell-center waypoints, or [] if the search finds no route.

    Grid entries mark unavailable cell centers.
    An optional segment_is_free(a, b) checks complete connections.
    Without it, use the original grid line-of-sight check.
    """
    if segment_is_free is None:
        def segment_is_free(a, b):
            return line_of_sight(grid, a, b)

    height = len(grid)
    if height == 0:
        raise ValueError("Planning grid is empty.")

    width = len(grid[0])
    if width == 0 or any(len(row) != width for row in grid):
        raise ValueError("Planning grid must be nonempty and rectangular.")

    if not is_free(grid, start):
        raise ValueError("Start is blocked or outside the grid.")
    if not is_free(grid, goal):
        raise ValueError("Goal is blocked or outside the grid.")

    if start == goal:
        return [start]

    # Entries contain (estimated total cost, cost so far, cell).
    frontier = [(distance(start, goal), 0.0, start)]
    costs = {start: 0.0}
    parents = {start: start}
    closed = set()

    while frontier:
        _, queued_cost, current = heapq.heappop(frontier)

        if current in closed or queued_cost > costs[current]:
            continue

        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parents[path[-1]])
            path.reverse()
            return path

        closed.add(current)
        x, y = current

        for offset_y in (-1, 0, 1):
            for offset_x in (-1, 0, 1):
                if offset_x == 0 and offset_y == 0:
                    continue

                neighbor = (x + offset_x, y + offset_y)

                if neighbor in closed or not is_free(grid, neighbor):
                    continue
                if not segment_is_free(current, neighbor):
                    continue

                # First consider reaching the neighbor through current.
                best_parent = current
                new_cost = costs[current] + distance(current, neighbor)

                # Theta*: try connecting directly through current's parent.
                ancestor = parents[current]
                if segment_is_free(ancestor, neighbor):
                    shortcut_cost = (
                        costs[ancestor] + distance(ancestor, neighbor)
                    )
                    if shortcut_cost <= new_cost:
                        best_parent = ancestor
                        new_cost = shortcut_cost

                if new_cost < costs.get(neighbor, math.inf):
                    costs[neighbor] = new_cost
                    parents[neighbor] = best_parent
                    priority = new_cost + distance(neighbor, goal)
                    heapq.heappush(
                        frontier, (priority, new_cost, neighbor)
                    )

    return []


def simplify_path(path: list[Cell]) -> list[Cell]:
    """Remove intermediate points on the same straight direction."""
    if len(path) < 3:
        return list(path)

    simplified = [path[0]]

    for index in range(1, len(path) - 1):
        a = simplified[-1]
        b = path[index]
        c = path[index + 1]

        ab_x, ab_y = b[0] - a[0], b[1] - a[1]
        bc_x, bc_y = c[0] - b[0], c[1] - b[1]

        cross = ab_x * bc_y - ab_y * bc_x
        dot = ab_x * bc_x + ab_y * bc_y

        if cross != 0 or dot <= 0:
            simplified.append(b)

    simplified.append(path[-1])
    return simplified

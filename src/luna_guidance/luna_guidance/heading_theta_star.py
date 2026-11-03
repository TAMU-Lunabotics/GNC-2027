"""
Experimental heading-aware search with Theta*-style shortcuts.

Forward straight movements and stationary turns only.
No claim of completeness or optimality for this modified search.
"""

import heapq
import itertools
import math


def heading_theta_star(
    checker,
    start,
    goal,
    start_yaw,
    turn_weight=0.0,
    max_expansions=20000,
    merge_turnable=False,
    allow_straight_departure=False,
):
    """
    Return a result dictionary containing status, path, and search counts.

    Positions and yaw use the world frame.
    Cost = distance in meters + turn_weight * turning in radians.
    Goal orientation is unconstrained.
    """
    start = tuple(start)
    goal = tuple(goal)
    if len(start) != 2 or len(goal) != 2:
        raise ValueError("Start and goal must contain two coordinates.")
    if not all(math.isfinite(v) for v in (*start, *goal, start_yaw)):
        raise ValueError("Positions and yaw must be finite.")
    if not math.isfinite(turn_weight) or turn_weight < 0:
        raise ValueError("Turn weight must be finite and nonnegative.")
    if type(max_expansions) is not int or max_expansions <= 0:
        raise ValueError("Expansion limit must be a positive integer.")
    if type(merge_turnable) is not bool:
        raise ValueError("merge_turnable must be Boolean.")
    if merge_turnable and turn_weight != 0.0:
        raise ValueError("Experimental merging requires zero turning weight.")
    if type(allow_straight_departure) is not bool:
        raise ValueError("allow_straight_departure must be Boolean.")

    geometry = checker.geometry

    # Valid only for this search, with fixed map and footprint settings.
    turn_cache = {}
    straight_cache = {}

    def turn_is_free(position):
        key = tuple(position)
        if key not in turn_cache:
            turn_cache[key] = checker.turn_is_free(position)
        return turn_cache[key]

    def straight_is_free(start_position, end_position):
        # Preserve direction and exact coordinates; do not round keys.
        key = (tuple(start_position), tuple(end_position))
        if key not in straight_cache:
            straight_cache[key] = checker.straight_is_free(
                start_position, end_position
            )
        return straight_cache[key]

    if not checker.pose_is_free(start, start_yaw):
        return {"status": "START_REJECTED", "path": [], "expanded": 0}
    if start == goal:
        return {"status": "PATH_FOUND", "path": [start], "expanded": 0}

    # Exact endpoints and optional off-grid straight-departure points.
    START = "start"
    GOAL = "goal"
    DEPARTURE = "departure"

    departure_step = geometry.resolution
    departure_dx = math.cos(start_yaw)
    departure_dy = math.sin(start_yaw)

    # A straight ray cannot travel farther than the map diagonal
    # while remaining inside the rectangular map.
    departure_limit = math.ceil(
        math.hypot(geometry.width, geometry.height)
    ) + 1

    def is_departure_node(node):
        return (
            isinstance(node, tuple)
            and len(node) == 2
            and node[0] == DEPARTURE
        )

    def position(node):
        if node == START:
            return start
        if node == GOAL:
            return goal
        if is_departure_node(node):
            distance = node[1] * departure_step
            return (
                start[0] + distance * departure_dx,
                start[1] + distance * departure_dy,
            )
        return geometry.cell_to_world(node)

    def angle_difference(a, b):
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def nearby_cells(point):
        x, y = geometry.world_to_cell(*point)
        return [
            (x + dx, y + dy)
            for dy in (-1, 0, 1)
            for dx in (-1, 0, 1)
            if geometry.contains((x + dx, y + dy))
        ]

    def state_key(incoming, node):
        if merge_turnable and (
            node == GOAL or turn_is_free(position(node))
        ):
            return (None, node)
        return (incoming, node)

    # Immutable records preserve the exact history used by each shortcut.
    # Record: (node, incoming_node, arrival_yaw, cost, parent_record_id)
    records = [(START, None, start_yaw, 0.0, None)]
    best = {state_key(None, START): 0.0}
    serial = itertools.count()
    frontier = [(math.dist(start, goal), next(serial), 0)]
    expanded = 0
    shortcut_updates = 0

    def offer(source_id, destination, shortcut=False):
        nonlocal shortcut_updates
        node, _, arrival_yaw, cost, _ = records[source_id]
        if destination == node:
            return

        a = position(node)
        b = position(destination)
        length = math.dist(a, b)
        if length <= 1e-12:
            return

        departure_yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        turn = angle_difference(arrival_yaw, departure_yaw)

        if turn > 1e-9 and not turn_is_free(a):
            return
        if not straight_is_free(a, b):
            return

        candidate_cost = cost + length + turn_weight * turn

        # Optional heuristic merging at positions with full turn clearance.
        key = state_key(node, destination)
        if candidate_cost >= best.get(key, math.inf):
            return

        best[key] = candidate_cost
        record_id = len(records)
        records.append(
            (destination, node, departure_yaw, candidate_cost, source_id)
        )
        priority = candidate_cost + math.dist(b, goal)
        heapq.heappush(frontier, (priority, next(serial), record_id))
        if shortcut:
            shortcut_updates += 1

    while frontier:
        _, _, record_id = heapq.heappop(frontier)
        node, incoming, _, cost, parent_id = records[record_id]
        if cost > best.get(state_key(incoming, node), math.inf):
            continue

        if node == GOAL:
            path = []
            cursor = record_id
            while cursor is not None:
                record = records[cursor]
                path.append(position(record[0]))
                cursor = record[4]
            path.reverse()
            return {
                "status": "PATH_FOUND",
                "path": path,
                "cost": cost,
                "expanded": expanded,
                "records": len(records),
                "turn_cache_entries": len(turn_cache),
                "straight_cache_entries": len(straight_cache),
                "shortcut_updates": shortcut_updates,
            }

        if expanded >= max_expansions:
            return {
                "status": "EXPANSION_LIMIT",
                "path": [],
                "expanded": expanded,
                "records": len(records),
                "turn_cache_entries": len(turn_cache),
                "straight_cache_entries": len(straight_cache),
            }
        expanded += 1

        if node == START:
            candidates = nearby_cells(start)

            # If turning is blocked, optionally depart along the
            # exact start yaw.
            if allow_straight_departure and not turn_is_free(start):
                candidates.append((DEPARTURE, 1))

        elif is_departure_node(node):
            candidates = nearby_cells(position(node))

            # Continue along the original heading. Each connection still
            # passes through offer(), including its full segment check.
            index = node[1]
            if index < departure_limit:
                candidates.append((DEPARTURE, index + 1))

        else:
            x, y = node
            candidates = [
                (x + dx, y + dy)
                for dy in (-1, 0, 1)
                for dx in (-1, 0, 1)
                if (dx or dy)
                and geometry.contains((x + dx, y + dy))
            ]

        # Test connection to the exact goal, not just its cell center.
        candidates.append(GOAL)

        for destination in candidates:
            offer(record_id, destination)

            # Check the parent shortcut independently: failure of the
            # current-node connection must not suppress a valid shortcut.
            if parent_id is not None:
                offer(parent_id, destination, shortcut=True)

    return {
        "status": "NO_ROUTE_IN_SEARCH",
        "path": [],
        "expanded": expanded,
        "records": len(records),
        "turn_cache_entries": len(turn_cache),
        "straight_cache_entries": len(straight_cache),
    }

"""
Azisly Hackathon -- Round 2 submission.

Only decide() is changed.  The protocol/JSON plumbing below is the supplied
starter-kit plumbing and must remain untouched for submission.
"""

import json
import sys
from collections import deque


def decide(sensors, memory):
    """Online, stateful autonomous maze solver.

    Design goals:
      * never depend on a maze file or hard-coded route;
      * maintain an odometry-based (x, y, heading) pose;
      * learn a persistent local wall/open-edge map;
      * explore systematically using BFS to the nearest known frontier;
      * tolerate encoder jitter;
      * treat a first zero distance reading as uncertain, which protects the
        solver against the ±1 distance noise used by the hardest mazes.
    """

    HEADINGS = ("N", "E", "S", "W")
    DELTA = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}

    def turn_dir(heading, direction):
        i = HEADINGS.index(heading)
        step = 1 if direction == "right" else -1
        return HEADINGS[(i + step) % 4]

    def opposite(direction):
        return HEADINGS[(HEADINGS.index(direction) + 2) % 4]

    def edge_state(x, y, direction):
        return memory["walls"].get((x, y), {}).get(direction)

    def mark_edge(x, y, direction, is_wall):
        """Record an edge from both cells, keeping the map symmetric."""
        value = bool(is_wall)
        memory["walls"].setdefault((x, y), {})[direction] = value
        dx, dy = DELTA[direction]
        nx, ny = x + dx, y + dy
        memory["walls"].setdefault((nx, ny), {})[opposite(direction)] = value

    # ------------------------------------------------------------------
    # Persistent state.  memory is supplied by the official runner and
    # survives for the complete maze attempt.
    # ------------------------------------------------------------------
    if not memory.get("initialized"):
        memory.clear()
        memory["initialized"] = True
        memory["x"] = 0
        memory["y"] = 0
        memory["heading"] = "N"
        memory["last_action"] = None
        memory["visited"] = {(0, 0)}
        memory["walls"] = {}
        # For an unknown edge whose current sensor says 0, retain the last
        # few samples.  A positive reading clears the samples.
        memory["sensor_samples"] = {}
        memory["collisions"] = 0

    if sensors.get("at_goal"):
        return "wait"

    # ------------------------------------------------------------------
    # 1) Interpret what the PREVIOUS action actually accomplished.
    # ------------------------------------------------------------------
    last_action = memory["last_action"]
    x = memory["x"]
    y = memory["y"]
    heading = memory["heading"]

    if last_action == "forward":
        rpm_l = sensors.get("rpm_left", 0)
        rpm_r = sensors.get("rpm_right", 0)

        # Encoder jitter is about ±5, so a tolerance is mandatory.
        moved = abs(rpm_l - 120) <= 8 and abs(rpm_r - 120) <= 8
        collided = (
            rpm_l == 0
            and rpm_r == 0
            and sensors.get("accel_fwd") == -2.0
        )

        if moved:
            dx, dy = DELTA[heading]
            x += dx
            y += dy
            memory["x"], memory["y"] = x, y
            memory["visited"].add((x, y))
        elif collided:
            memory["collisions"] += 1
            # A physical impact is definitive evidence of a wall.
            mark_edge(x, y, heading, True)

    elif last_action == "turn_left":
        rpm_l = sensors.get("rpm_left", 0)
        rpm_r = sensors.get("rpm_right", 0)
        if abs(rpm_l + 60) <= 8 and abs(rpm_r - 60) <= 8:
            heading = turn_dir(heading, "left")
            memory["heading"] = heading

    elif last_action == "turn_right":
        rpm_l = sensors.get("rpm_left", 0)
        rpm_r = sensors.get("rpm_right", 0)
        if abs(rpm_l - 60) <= 8 and abs(rpm_r + 60) <= 8:
            heading = turn_dir(heading, "right")
            memory["heading"] = heading

    memory["x"], memory["y"] = x, y
    memory["visited"].add((x, y))

    # ------------------------------------------------------------------
    # 2) Translate relative sensors into absolute compass directions.
    # ------------------------------------------------------------------
    readings = {
        heading: sensors["dist_front"],
        turn_dir(heading, "left"): sensors["dist_left"],
        turn_dir(heading, "right"): sensors["dist_right"],
    }

    # ------------------------------------------------------------------
    # 3) Update the local map.
    #
    # A positive distance proves the adjacent cell is open, but on the noisiest
    # maze a wall can occasionally appear as distance 1.  Unknown distance-1
    # edges therefore get one confirmation sample before we drive into them.
    # Unknown zero edges are also confirmed.  Once an edge is known, later noisy
    # readings never overwrite it.
    samples = memory["sensor_samples"]
    pending = False

    for direction, reading in readings.items():
        state = edge_state(x, y, direction)
        key = (x, y, direction)

        if state is not None:
            samples.pop(key, None)
            continue

        history = samples.setdefault(key, [])
        history.append(1 if reading > 0 else 0)
        if len(history) > 2:
            history.pop(0)

        # Need two consistent samples for the ambiguous adjacent edge.
        if len(history) < 2:
            pending = True
            continue

        if history[-1] == 1 and history[-2] == 1:
            mark_edge(x, y, direction, False)
            samples.pop(key, None)
        elif history[-1] == 0 and history[-2] == 0:
            mark_edge(x, y, direction, True)
            samples.pop(key, None)
        else:
            # Conflicting samples: keep the latest one and obtain another.
            history[:] = [history[-1]]
            pending = True

    if pending:
        memory["last_action"] = "wait"
        return "wait"

    # ------------------------------------------------------------------
    # 4) BFS through the discovered graph.
    #
    # The goal location is intentionally NOT assumed.  Instead, we drive to
    # the nearest cell that still has an unknown edge.  This is systematic
    # exploration and guarantees progress through any finite connected maze
    # as long as the sensed local map is correct.
    # ------------------------------------------------------------------
    start = (x, y)

    def open_neighbors(cell):
        cx, cy = cell
        for direction in HEADINGS:
            if edge_state(cx, cy, direction) is False:
                dx, dy = DELTA[direction]
                yield (cx + dx, cy + dy), direction

    queue = deque([start])
    previous = {start: None}
    target = None

    while queue:
        current = queue.popleft()

        if current not in memory["visited"]:
            target = current
            break

        edges = memory["walls"].get(current, {})
        if any(edges.get(direction) is None for direction in HEADINGS):
            target = current
            break

        for nxt, direction in open_neighbors(current):
            if nxt not in previous:
                previous[nxt] = (current, direction)
                queue.append(nxt)

    # Reconstruct the absolute-direction route to the selected target.
    route = []
    if target is not None and target != start:
        current = target
        while previous[current] is not None:
            parent, direction = previous[current]
            route.append(direction)
            current = parent
        route.reverse()

    # At the current frontier, choose an unknown direction that the live
    # sensor currently says is open.  Prefer straight, then left, right,
    # then reverse to reduce unnecessary turns.
    if target == start:
        edges = memory["walls"].get(start, {})
        candidates = [
            direction for direction in HEADINGS
            if edges.get(direction) is None and readings.get(direction, 0) > 0
        ]
        preference = [
            heading,
            turn_dir(heading, "left"),
            turn_dir(heading, "right"),
            opposite(heading),
        ]
        if candidates:
            route = [next(d for d in preference if d in candidates)]

    # Safety fallback: if the map has no reachable frontier, prefer a known-open
    # edge.  If even that is unavailable, rotate to expose the unsensed rear
    # direction; the robot cannot observe its rear wall with the current pose.
    if not route:
        safe = [d for d in HEADINGS if edge_state(x, y, d) is False]
        if safe:
            preference = [
                heading,
                turn_dir(heading, "left"),
                turn_dir(heading, "right"),
                opposite(heading),
            ]
            route = [next(d for d in preference if d in safe)]
        else:
            unknown = [d for d in HEADINGS if edge_state(x, y, d) is None]
            if unknown:
                # A right turn changes which absolute edge is in the front/side
                # sensor set.  Re-plan immediately on the next tick.
                memory["last_action"] = "turn_right"
                return "turn_right"

    # ------------------------------------------------------------------
    # 5) Convert the next desired compass direction into one legal action.
    # ------------------------------------------------------------------
    if route:
        desired = route[0]

        if desired == heading:
            memory["last_action"] = "forward"
            return "forward"

        if desired == turn_dir(heading, "right"):
            memory["last_action"] = "turn_right"
            return "turn_right"

        if desired == turn_dir(heading, "left"):
            memory["last_action"] = "turn_left"
            return "turn_left"

        # Desired direction is behind us.  Either 90° turn is equally good;
        # use right consistently and re-plan on the next tick.
        memory["last_action"] = "turn_right"
        return "turn_right"

    # No known safe move yet.  Obtain another sensor sample rather than
    # intentionally risking a collision.
    memory["last_action"] = "wait"
    return "wait"


# =============================================================================
# DO NOT EDIT BELOW THIS LINE
# This is the plumbing that talks to the grader. Changing it will break your
# submission and score you zero.
# =============================================================================

def _main():
    print(json.dumps({"ready": True}), flush=True)
    memory = {}
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        sensors = json.loads(line)
        action = decide(sensors, memory)
        print(json.dumps({"action": action}), flush=True)
        if sensors.get("at_goal"):
            break


if __name__ == "__main__":
    _main()
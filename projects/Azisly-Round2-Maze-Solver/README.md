# Azisly Hackathon — Round 2 Maze Solver

An autonomous, stateful micromouse controller for an unknown maze. The solver learns a local wall/open-edge map from the live sensor stream, maintains an odometry-based pose, and uses breadth-first search to explore the nearest known frontier.

## What this version improves

- Persistent `(x, y, heading)` pose tracking.
- Symmetric wall/open-edge map.
- BFS exploration instead of a simple wall follower.
- Encoder-jitter tolerance when resolving completed forward/turn actions.
- Two-sample confirmation for ambiguous sensor readings.
- Defensive collision handling and safe fallbacks.
- No maze file, hard-coded route, network access, or third-party runtime dependency.

## Included

```text
Azisly-Round2-Maze-Solver/
├── robot.py
├── tests/test_robot.py
└── README.md
```

The original competition starter-kit files are intentionally not copied into this portfolio folder. `robot.py` retains the grader-facing protocol required by the submission.

## Local validation

The submission passed the supplied Round 2 self-test and solved all four locally available practice mazes in the provided simulator with zero collisions.

| Maze | Result | Ticks | Collisions |
|---|---|---:|---:|
| p01 | SOLVED | 74 | 0 |
| p02 | SOLVED | 144 | 0 |
| p03 | SOLVED | 163 | 0 |
| p04 | SOLVED | 118 | 0 |

## Constraints

The controller targets Python 3 with the standard library only, a JSON line protocol, live sensors, and no runtime internet or external packages.

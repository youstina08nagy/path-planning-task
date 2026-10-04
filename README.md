## Path Planning Assignment


### Task Description

#### Part 1

You are given up to two cones (sometimes only one, or none), each with coordinates `(x, y)` and a `color` flag:
- `color == 0`: yellow cone (right side of the track)
- `color == 1`: blue cone (left side of the track)

You also receive the car pose `(x, y, yaw)`. Your goal is to return a sequence of path points `(x, y)` in world coordinates, representing a drivable route that stays between the left (blue) and right (yellow) boundaries. The path will be visualized by the tester.

Implement your algorithm in:

```
src/path_planning.py  ->  class PathPlanning.generatePath(self) -> Path2D
```

The environment already runs with a basic straight-line placeholder. Replace it with your solution.

#### Part 2

What if you were given three cones on one side of the track?
- Implement your approach to use the new givens.
- Add new test cases to test your approach in `src/scenarios.py`.
- Explain why you chose this solution and what could be its limitations.

### Quick Start

1. Install Python 3.9+.
2. Create a virtual environment (recommended) and install dependencies:

```
python -m venv .venv
source .venv/bin/activate  # macOS/Linux
# .venv/Scripts/activate   # Windows
pip install -r requirements.txt
```

3. Run a scenario:

```
python -m src.run --scenario 1
```

Available scenarios: numeric names `1`..`20` (each with up to 2 blue and 2 yellow cones placed on a 5x5 grid; car at (0,0) with varying yaw). Use `--scenario N` to select.

### Files Overview

- `src/models.py`: data classes for `Cone`, `CarPose`, and `Path2D` alias.
- `src/path_planning.py`: contains `PathPlanning` where you implement `generatePath`.
- `src/tester.py`: simple Matplotlib visualizer that plots cones, the car, heading, and the path.
- `src/scenarios.py`: prebuilt scenarios for testing.
- `src/run.py`: CLI to run a scenario and visualize the result.

### What to Submit

- Upload your solution to github and submit your repository link.
- Don't forget to add documentation, explaining your solution.

### Solution Documentation

#### 1. Approach and Architecture
- **Boundary Separation & Ordering**: Cones are partitioned by color into left boundary (`color == 1`, Blue) and right boundary (`color == 0`, Yellow). Cones on each side are ordered based on proximity to the vehicle and track flow.
- **Midpoint and Virtual Gate Extraction**:
  - When cones exist on both sides, cones are matched into track gates based on nearest neighbors. The centerline waypoints are set to gate midpoints, and the track width is computed directly from cone pair distances. Unmatched cones are projected inward using the local track normal scaled by half-track width.
  - When cones are visible on only one side (1, 2, or 3 cones), local polyline tangents are computed along the visible boundary. Points are offset inward by the nominal half-track width ($1.0\,\text{m}$) perpendicular to the track tangent (left for yellow cones, right for blue cones).
  - When no cones are present, the planner projects directly along the vehicle's initial heading.
- **Hermite Spline Interpolation**:
  - The path begins at the vehicle's exact position $(x, y)$ with tangent matching the car's heading $\text{yaw}$.
  - A cubic Hermite spline connects the vehicle pose through all target waypoints to ensure $C^1$ continuity and smooth curvature without abrupt steering transitions.
  - The spline extends along the final tangent to satisfy the total length requirement.
- **Uniform Arc-Length Resampling**:
  - The continuous spline is resampled uniformly with a step size of $\Delta s = 0.25\,\text{m}$ ($\le 0.5\,\text{m}$) to achieve a total path length of $7.0\,\text{m}$ (within $5\text{--}10\,\text{m}$).

#### 2. Part 2: Handling Three Cones on One Side
- The algorithm naturally supports arbitrary numbers of cones (including 3 cones on one side, or 3 on both).
- For 3 cones along a boundary, the algorithm constructs a piecewise polyline through the 3 cone coordinates in travel order, computes local forward segment tangents, and offsets each point inward by the half-track width along the normal vector.
- New test cases added in `src/scenarios.py`:
  - `Scenario 21`: 3 yellow cones along a curved right boundary.
  - `Scenario 22`: 3 blue cones along a curved left boundary.
  - `Scenario 23`: 3 yellow cones on the right with 1 blue cone on the left.
  - `Scenario 24`: 3 blue cones on the left with 2 yellow cones on the right.
  - `Scenario 25`: 3 blue cones and 3 yellow cones forming a full 3-gate track segment.

#### 3. Rationale for Solution
- **Continuity & Drivability**: By enforcing the vehicle's initial pose and heading at the start of the spline, the planned trajectory is dynamically feasible and smooth for a controller (Pure Pursuit, Stanley, or MPC) to follow.
- **Robustness Across Sparse & Asymmetric Geometries**: Handles all combinations (0, 1, 2, or 3 cones on either side) through a unified geometric formulation without hardcoded special cases.
- **Zero Heavy Dependencies & High Performance**: Operates using pure standard Python math, executing all 25 scenarios in milliseconds.

#### 4. Limitations
- **Fixed Half-Width Assumption for Single-Sided Detections**: When cones are only visible on one side, a nominal track width ($2.0\,\text{m}$) is assumed. If the actual track width varies significantly on single-sided sections, the path may not be perfectly centered.
- **Local Horizon**: The planner relies on immediate local cone visibility. On sharp hairpins where far cones appear across the track, global SLAM or Delaunay triangulation would be needed.

### Author's Note
- Keep it simple.
- This task was chosen to give you a sense of how things work, you don't have to implement complex algorithms, read research papers, or spend days thinking about it, just implement the simplest approach you can think of that can return a correct path.
- You are free to assume any missing information, but mention it in your submission.
- Try to solve as many cases as you can, but you don't have to solve all the problems that can ever exist, just try your best.
- Reminder: This problem is based on FSAI competition (a simplified version), it does not represent how path planning works everywhere it just provides an example to give you a better understanding of what path planning can look like.
- Enjoy :)

## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
- A `Planning` state is added, in which a path planning is performed right after the drone is armed.
- The `plan_path` method creates waypoints from the start position to the goal, by performing the following steps:
  - Create a 2.5D grid configuration (`create_grid` in `planning_utils.py`)
  - Run an A* algorithm (`a_star` function in `planning_utils.py`) to find out the shortest path from the given start position to the goal.
- Takeoff and fly throgh the waypoints, the same implementation as `backyard_flyer_solution.py`

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

- Read the first line of the csv file using `readline` function
- Extract the numbers of latitude/longtitude from text using RegExp pattern matching

#### 2. Set your current local position

Invoke `global_to_location` function to convert the global current location into a local one, by passing `self.global_position` and `self.global_home` as parameters.

#### 3. Set grid start position from local position

Convert the current location into grid coordinates, and use it as start point of the path.

#### 4. Set grid goal position from geodetic coords

- Use `global_to_location` function to convert a given global cooridinate into a local position.
- Convert the local position into grid coordinate, and use it as the goal position.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

Include diagonal motion:

- Define diagonal actions: `north east`, `north west`, `south east` and `south west`, and the cost / delta for each of them.
- Modify `valid_actions` funcion, to remove corresponding actions when any diagonal cell is off the grid or is infeasible.

#### 6. Cull waypoints

Here the Bresenham algorithm is used to detect redundent waypoints, implemented in function `prune_path_bresenham`.

- Iterates the path, checking 3 points in each step
- Run Bresenham algorithm on point#1 and point#3, which produces cells on the line from `p1` to `p3`
- If point#2 is on the line, remove it
- And now, point#3 becomes the second point, run the iteration again, until no more points can be removed
- Move to next point on the path, do the same checking
- Finally we can get the pruned path

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

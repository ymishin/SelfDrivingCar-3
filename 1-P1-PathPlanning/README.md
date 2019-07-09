# **Path Planning**
---

**The goal of this project is to implement a path planning algorithm to safely navigate around a virtual highway with other traffic.**

---

The path planning algorithm is implemented in **PlanBehaviorAndCalculateTrajectory** method of **Planner** class and consists of two steps - behavior planning and trajectory generation.

### Behavior Planning

At the first step of behavior planning, all cars from fusion data are analyzed and their positions forward in time are predicted. The goal of this analysis is to find out if our car is constrained by other cars to **(a)** continue drive forward and to **(b)** change lane. At the next step, the behaviour of our car is planned. The logic is pretty straighforward and can be summarized as follows:

* If there is a car ahead of our car -> try to change lane if possible. Changing lane to the left is preffered.
* If there is no car ahead -> try to change lane to the target lane (by default it is a middle lane).

If our car follows a car ahead, the reference velocity of our car is set to the velocity of a car ahead. Otherwise the reference velocity is set to predefined maximum value.

The result of behaviour planning step is reference velocity and lane to drive for our car.

### Trajectory Generation

Trajectory generation is based on spline interpolation using external library (https://kluge.in-chemnitz.de/opensource/spline/).

At the first step, 5 reference points for spline interpolation are constructed, these are:

* 2 points from previous path or, if previous path is almost empty, current position of the car and 1 point backward in time.
* 3 points ahead with predetermined step. These are constructed first in Frenet coordinates taking into account desired lane determined during behaviour planning step. Then transformation to Cartesian coordinates is performed.

After these points are constructed, they are transformed from global to car coordinates to facilitate further calculations.

At the next step, trajectory to follow is generated as follows:

* All points left in previous path are copied to the new path.
* Target distance is calculated based on predefined planning horizon.
* Spline interpolation is performed and next points to follow are calculated in local car coordinates.
* Points to follow are transformed from car to global coordinates.

When calculating coordinates of points during spline interpolation, velocity is gradually adjusted to approximate reference velocity determined during behavior planning step.

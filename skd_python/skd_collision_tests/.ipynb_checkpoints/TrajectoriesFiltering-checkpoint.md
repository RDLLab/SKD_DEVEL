## Trajectory Filtering

When decided how to modify a given trajectory (morphing or filtering), two things have to be considered
- The initial distance between the pedestrian and the car has to be greater than the stopping distance of the safest controller considered.
- There must be an interaction between the safe trajectory and the assesed car.


## Problem:
Although a sufficient distancing can be ensured by allowing the two agents to start with a relative distance to the lowest point in the pedestrian trajectory,

A proper interaction cannot be ensured this way.
How do we ensure interaction with the assessed controller?

## Problem: Ensuring Interaction
Given a point of interaction, such point would have to be one such that the car can reach the point of interaction in time (discretized state update), and 
at the same time, allow a sufficient gap with the safest controller ()

The point of interaction would have to be such that the car can reach the point of interaction around the same time index as the "pedestrain safe trajectory".


## Experiment parameter constraint:
With the initial distancing constraint (the safest controller's stopping distancing is ~ 17m (~ 6.8 steps assuming maximum speed)). 
Car Stop time = 2.38s ~ (9 simulations steps)


## Constraint on size of the safe trajectory
- Even if the car starts deceleration from the first step, it would required about 9 steps to come to a full stop. This means that a safe trajectory (with similar discretization), has to have at least 9 simulations steps.....

** CAN WE DEFINE CHARACTERISTICS OF A SAFE TRAJECTORY THAT WOULD ENSURE INTERACTION WITH THE VEHICLE **


## Decoupling basic safe controllers:
If basic safe controllers are decoupled, an interaction point would have to assigned to each of the car controllers (when starting at different locations, or a single point, (when starting at same location)). However, a systematic way of ensuring this interactions is still needed.


## Enforcing constraints:
- One way to enforce the constraint would be to lock in a point of interest in the safe trajectory and artificially augment or displace the safe trajectory 
in order to fit the requirements. However, in order to make a trajectory fit the requirements for a valid safe_trajectory must be met.

- Current constraints are:
Given an initial relative distancing, an interaction period will ocur between the "Nth and the Nth +k" indices. 
To satisfy the interaction constraint, this means that a safe trajectory must be at least "N points in length", and that the pedestrian must be within 
the width of the car within those periods.


## Using aleatory safe trajectories:
If requirements are not properly defined for safe trajectories, how can we ensure that a not so biased set of safe trajectories is being considered?
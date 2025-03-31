# Rivelin-Robotics

### I worked under the assumption that the +ve angle inputs were to indicate clock-wise rotatation.
### This assumption is based on the test cases 
#### {-45, 90, -135, 45, 0} Collision
#### {90, 45, -30, 0, 50} Collision
### In the second test case, the second value represents the rotation about pivot U2. A rotation of 90 degrees would cause the arm to collide with the floor.

## Part 1 Forward kinematics

Determining the coordinates of the effectors: The coordinates of each joint are computed, starting from the base origin frame. Using trigonometric transformations (sine and cosine functions), each joint’s position is determined relative to the previous joint.

Then, using the given knowledge of the lengths of the arm segments, the position of the lipstick holder and the final end effector position are determined and returned as (x,y,z) coordinate.

The U1 angle rotation (about z-axis) is handled last, as the only coordiantes that are affected are (x,y) - hence the x-coordinate had to be determinied first.

Regarding the collision detection: Since links are treated as having zero thickness, collision detection is performed by checking whether any part of the arm moves below the floor (z = 0).

## Part 2 Path planning
### Trajectory Determination

The goal is to calculate a trajectory where all joints complete their motion simultaneously. I have achieved this using the following steps:

#### Step 1: Compute the Required Motion for Each Joint
The difference between the target and start positions is determined for each joint.

#### Step 2: Determine the Required Time for Motion Completion

Using kinematic equations, I have determined the time needed for each joint to reach its target. I considered that depending on the scale of the angle change, the join may or may not reach its maximum speed, hence:

If the maximum speed is not reached, time is computed using constant acceleration.

If the maximum speed is reached, the time is split into acceleration, constant velocity, and deceleration phases.

The longest computed time among all joints is selected as the total trajectory duration to ensure synchronised movement.

#### Step 3: Generate Intermediate Joint States

The number of steps required is determined using the system’s update rate of 150Hz.
Intermediate joint states are determined linearly over the computed time period, which are then all added to an array.
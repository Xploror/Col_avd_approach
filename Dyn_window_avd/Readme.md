## Dynamic Window Approach (Reactive Collision Avoidance)

### Theory
Dynamic Window Approach (DWA) is a simple local path planning method that specializes in real-time obstacle avoidance. It computes the real-time optimal linear and angular kinematics based on the following objectives:
* Avoiding static and moving obstacles
* Reaching the goal position
* Maintaining the kinematics within the set physical constraints on the robot actuators

At each timestep, the robot knows about its current position, velocity and acceleration limits and with such known data it samples combination of linear and angular velocities for a given predictive time window. It is assumed that the robot is capable of predicting the future position and velocity of the obstacles in the environment as well. Out of all the non-colliding trajectories, the robot chooses the most favourable trajectory that is given a score based on the safety and locally quicker path to the goal. Usually its a trade-off between the two metrics.

### Methodology

### Results

The results look promising! Given the maximum linear and turning velocities, the robots indicated in blue markers are clearly able to locally navigate itself from the starting point (red) to a common goal (green). Moreover, due to the limits in acceleration as well, the robots transition smoothly whenever it senses a clear path to the goal. It is assumed that the obstacles have a fixed velocity and the robots already have the knowledge about it. From the figures, it can be observed that some robots would completely stop when sensing an obstacle in proximity and would rather wait for the obstacle to leave rather than taking a more expensive detour. The hyperparameters like prediction horizon, velocity limits and all teh associated weights can be tuned to have a realistic experiment that can give interesting results!

<div align="center">
  <video src="https://github.com/user-attachments/assets/d05f1151-d931-4609-9c8c-65c10c9bdd79" width="100" />
</div>

<div align="center">
  <video src="https://github.com/user-attachments/assets/877e0a3a-4394-40b8-842a-ba7f6c54158e" width="100" />
</div>








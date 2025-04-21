## Dynamic Window Approach (Reactive Collision Avoidance)

### Theory
Dynamic Window Approach (DWA) is a simple local path planning method that specializes in real-time obstacle avoidance. It computes the real-time optimal linear and angular kinematics based on the following objectives:
* Avoiding static and moving obstacles
* Reaching the goal position
* Maintaining the kinematics within the set physical constraints on the robot actuators

At each timestep, the robot knows about its current position, velocity and acceleration limits and with such known data it samples combination of linear and angular velocities for a given predictive time window. It is assumed that the robot is capable of predicting the future position and velocity of the obstacles in the environment as well. Out of all the non-colliding trajectories, the robot chooses the most favourable trajectory that is given a score based on the safety and locally quicker path to the goal. Usually its a trade-off between the two metrics.

### Methodology

### Results

<div align="center">
  <video src="https://github.com/user-attachments/assets/d05f1151-d931-4609-9c8c-65c10c9bdd79" width="100" />
</div>

<div align="center">
  <video src="https://github.com/user-attachments/assets/877e0a3a-4394-40b8-842a-ba7f6c54158e" width="100" />
</div>








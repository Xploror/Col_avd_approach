function cost = calc_path_cost(main, cl_dist, obs_rad, g_init, goal_pos)
% This function outputs optimal path out of set of viable paths along with
% the cost
% Optimal path includes ---> [vel, ang_vel]
% main.vel is the final velocity after prediction timestep with same
% magnitude but rotated

goal_coeff = 1000;
vel_coeff = 10;
obs_coeff = 10;
ang_coeff = 10;
theta_coeff = 100;
goal_dist_current = sqrt(sum((goal_pos - main.pos).^2)); % Finds main's distance from goal after prediction time interval

if cl_dist < (main.radius + obs_rad) + 0.5
    obs_cost = -Inf;
else
    obs_cost = cl_dist;
end

cost = goal_coeff*(g_init - goal_dist_current) + vel_coeff*(sqrt(sum((main.vel).^2))) + obs_coeff*(obs_cost) + ang_coeff*(main.max_ang_vel - abs(main.ang_vel));
end
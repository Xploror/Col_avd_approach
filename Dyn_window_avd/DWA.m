clc;clear;

%% Parameters

%%%%%%%%%%%%%%%%%%%%% Algorithm Main Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.1;
horizon = 10;
det_t = dt*horizon;   % Total time interval of the prediction 
% goal_coeff = 10;
% vel_coeff = 5;
% obs_coeff = 5;
% ang_coeff = 5;
distance_thresh = 5;  % 5 units
del_ang_vel = 0.08;   % rad/s
del_vel = 0.08;  % units/s  0.05
goal_pos = [0,-5]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%% Playground settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_obs = 30;
n_main = 5;
min_b = -5;  % Minimum of boundary in coordinate system (unit)
max_b = 5;   % Minimum of boundary in coordinate system (unit)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initializing classes

% Body Class
% paths, max_vel and max_ang_vel is not defined for Obstacles, its only for Main

body.pos = [];
body.vel = [];
body.ang_vel = [];   % direction in z direction if 2D maneuvering in xy plane
body.radius = [];
body.paths = [];     % contains final velocity and angular_velocity choosen and cost ----> [[vel, ang_vel], cost]
body.bestpath = [];
body.max_vel = [];
body.max_ang_vel = [];
body.max_acc = [];
body.max_ang_acc = [];

% Trajectory class
traj.cost = [];
traj.fin_pos = [];
traj.fin_theta = [];
traj.vel = [];
traj.ang_vel = [];

% Obstacle initialization
Obs = repmat(body, n_obs, 1);
for i=1:n_obs
    Obs(i).radius = 0.3;
    Obs(i).pos = (max_b-min_b)*rand(2,1) + min_b;
    Obs(i).vel = (0.2)*rand(2,1) - 0.1;
    %Obs(i).vel = [0 0]';
    Obs(i).ang_vel = 0;
end

% Main Body initialization
main = repmat(body, n_main, 1);
for i=1:n_main
    main(i).radius = 0.2;
    %main(i).pos = [5,10]';
    main(i).pos = (20)*rand(2,1) - 10;
    main(i).vel = [0,0]';
    main(i).theta = 0;
    main(i).ang_vel = 0;
    %%%%%%%%%%%%%%%%%%% Actuator Limits %%%%%%%%%%%%%%%%%%%%%%%%%
    main(i).max_acc = 7;  %7  % 10 unit/s^2
    main(i).max_ang_acc = 110*pi/180; % 150 rad/s^2
    main(i).max_vel = 3; %10
    main(i).max_ang_vel = 0.5;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end



% Plotting
t = 0;
while t<200
   
   % Provides next timestep optimal path for main bodies and move_ahead() using vel and ang_vel components updated   
   main = position_estimator(main, Obs, det_t, dt, del_vel, del_ang_vel, goal_pos);
   main = move_ahead(main, dt, dt);
   [sqrt(sum(main(1).vel.^2)) main(1).ang_vel]
   % Estimating Obstacle position after sampling timestep 
   Obs = move_ahead(Obs, dt, dt);
   
   O_pos = [Obs.pos];
   m_pos = [main.pos];
   p = plot(O_pos(1,:), O_pos(2,:), 'ok', m_pos(1,:), m_pos(2,:), 'ob');
   p(1).MarkerSize = 20*Obs(1).radius;
   p(2).MarkerSize = 20*main(1).radius;
   xlim([-20 20])
   ylim([-20 20])
   %line([0 50*main(1).vel(1)], [0 50*main(1).vel(2)]);
   pause(0.001)
   t = t + 1;
end

% t = 0;
% while t<300
%     x_obs = [];
%     y_obs = [];
%     for i=1:n_obs
%         body(i).pos = body(i).pos + body(i).vel*dt;
%         if body(i).pos(1) > max_b
%             body(i).pos(1) = min_b;
%         elseif body(i).pos(2) > max_b
%             body(i).pos(2) = min_b;
%         elseif body(i).pos(1) < min_b
%             body(i).pos(1) = max_b;
%         elseif body(i).pos(2) < min_b
%             body(i).pos(2) = max_b;
%         end
%             
%         x_obs = [x_obs body(i).pos(1)];
%         y_obs = [y_obs body(i).pos(2)];
%     end
%     plot(x_obs, y_obs, 'or')
%     xlim([-5 5])
%     ylim([-5 5])
%     pause(0.01)
%     t = t + 1;
% end
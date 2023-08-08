function m = position_estimator(m, o, det_t, dt, dv, d_angv, g_pos)
% m(i).paths shape ---> [length(v_span)*length(angv_span), 3]
% index is the nearest obstacle index for array of Obstacles "o"
% This funciton modifies only vel and angular_vel data for main objects

def_vel_dir = [0 1]';

% Predict the future position of all the obstacles  // ***later change it for only proximity based obstacles***  
o = move_ahead(o, det_t, dt);

for i=1:length(m)
    
    m(i).paths = [];
    
    goal_dist_init = sqrt(sum((g_pos - m(i).pos).^2));
    v_span = max((sqrt(sum((m(i).vel).^2)) - m(i).max_acc*dt), -m(i).max_vel) : dv : min((sqrt(sum((m(i).vel).^2)) + m(i).max_acc*dt), m(i).max_vel);
    angv_span = max((m(i).ang_vel - m(i).max_ang_acc*dt), -m(i).max_ang_vel) : d_angv : min((m(i).ang_vel + m(i).max_ang_acc*dt), m(i).max_ang_vel);
    
    % These loops will calculate all the paths for individual sample time interval and save it to body.paths | After this the paths would beselected based on costs 
    for j=1:length(v_span)
        for k=1:length(angv_span)
            
            main_temp = m(i); % this will be here because for every path I want the current timestep kinematics rather than previous paths kinematics updated in main_temp (Important !!)
            % This will take the main body and the combination of vel and
            % ang_vel and calc its path and its cost w.r.t obstacles
            [main_temp, closest_dist, index] = predict_path(main_temp, o, v_span(j), angv_span(k), det_t, dt, def_vel_dir); % Updates body.pos and body.vel and calculates closest distance with its index
            m(i).paths = [m(i).paths; [v_span(j), angv_span(k), calc_path_cost(main_temp, closest_dist, o(index).radius, goal_dist_init, g_pos)]];  % Individual body will have length(v_span)*length(angv_span) number of viable paths 
        end 
    end
    
    if goal_dist_init > 0.5
        [cost_opt, ind] = max(m(i).paths(:,3));  %Pick the path with maximum cost 
        opt_param = m(i).paths(ind,1:2); % contains velocity and angular_velocity component respectively
        if sqrt(sum(m(i).vel)) ~= 0
            m(i).vel = opt_param(1)*m(i).vel/sqrt(sum(m(i).vel.^2));
        else
            m(i).vel = opt_param(1)*def_vel_dir;
        end
        %m(i).vel = opt_param(1);
        m(i).ang_vel = opt_param(2);
    else
        m(i).vel = [0 0]';
        m(i).ang_vel = 0;
    end
    
end

end
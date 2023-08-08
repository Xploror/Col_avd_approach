function [body, closest_dist, index] = predict_path(body, obs, vel, angv, det_t, dt, def_dir)
% det_t is the prediction time interval
% At this stage, obs is already obstacles, det_t in future...
% vel and angv are velocity and angular_velocity choosen for the path (scalars)
% body is a single body object

R = @(th) [cos(th) -sin(th); sin(th) cos(th)];  % Rotational matrix

body.ang_vel = angv;
if sqrt(sum(body.vel.^2)) ~= 0
    body.vel = vel*body.vel/sqrt(sum(body.vel.^2));   % This is a velocity vector just with magnitude change | if vel is neg then change in velocity direction otherwise same dir.
else
    body.vel = vel*def_dir;    % Just start pointing towards +y axis
end
linear_vel = body.vel;

% Get the final position of the body after prediction time interval
for t=0:dt:det_t    
    linear_vel = R((angv*dt))*linear_vel;
    body.pos = body.pos + linear_vel*dt;     
end
% find closest obstacle 
[closest_dist, index] = find_closest(body, obs);

end
function O = move_ahead(O, t, dt)

R = @(th) [cos(th) -sin(th); sin(th) cos(th)];
min_b = -5;  % Minimum of boundary in coordinate system (unit)
max_b = 5;   % Minimum of boundary in coordinate system (unit)

for i = 0:dt:t
    for j = 1:length(O)
        theta = O(j).ang_vel*dt;
        O(j).vel = R(theta)*O(j).vel;
        O(j).pos = O(j).pos + O(j).vel*dt;
        
%         O(j).pos = O(j).pos + O(j).vel*dt;
%         if O(j).pos(1) > max_b
%             O(j).pos(1) = min_b;
%         elseif O(j).pos(2) > max_b
%             O(j).pos(2) = min_b;
%         elseif O(j).pos(1) < min_b
%             O(j).pos(1) = max_b;
%         elseif O(j).pos(2) < min_b
%             O(j).pos(2) = max_b;
%         end
    end
end

end
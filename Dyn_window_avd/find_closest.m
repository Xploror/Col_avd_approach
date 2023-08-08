function [closest_dist, index] = find_closest(b, o)
% Closest_dist is a scalar

closest_dist = Inf;
index = 0;

for i=1:length(o)
    
    if sqrt(sum((o(i).pos - b.pos).^2)) - o(i).radius - b.radius < closest_dist
        closest_dist = sqrt(sum((o(i).pos - b.pos).^2));
        index = i;
    end

end
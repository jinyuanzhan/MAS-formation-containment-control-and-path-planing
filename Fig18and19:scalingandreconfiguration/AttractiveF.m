
function F_att = AttractiveF(current, goal, zeta, F_max)
    % Calculate attractive force between an object and its goal
    % F_max is the maximum allowable attractive force magnitude
    F_att = zeros(1, 2); % Initialize the attractive force vector
    
    for i = 1:size(current,1)
        dx = -zeta * (current(i,1) - goal(1,1));
        dy = -zeta * (current(i,2) - goal(1,2));
        
        % Calculate the magnitude of the force vector
        F_mag = sqrt(dx^2 + dy^2);
        
        % If the magnitude exceeds F_max, scale it down
        if F_mag > F_max
            dx = dx * F_max / F_mag;
            dy = dy * F_max / F_mag;
        end
        
        F_att(1) = dx;
        F_att(2) = dy;
    end
end

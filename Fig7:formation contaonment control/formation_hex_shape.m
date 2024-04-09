

function [form_coord, delta_x, delta_y] = formation_hex_shape(x_leader, y_leader, spacing)
    % Initialize coordinates for 4 followers in a rectangle shape
    form_coord = zeros(4, 2); 
    deltas = zeros(4, 2); 

    % Define the rectangle shape relative positions from the leader
    % Assuming the leader is at the center of the rectangle,
    % we only define positions for the middle left, middle right,
    % top middle, and bottom middle positions
    % form_coord(1, :) = [x_leader - (sqrt(3)/2)*spacing, y_leader + spacing/2 + 30 ]; % Middle left
    % form_coord(2, :) = [x_leader - (sqrt(3)/2)*spacing - 10, y_leader - spacing/2]; % Bottom middle
    % form_coord(3, :) = [x_leader + (sqrt(3)/2)*spacing, y_leader - spacing/2 + 15]; % Middle right
    % form_coord(4, :) = [x_leader + (sqrt(3)/2)*spacing - 10, y_leader + spacing/2]; % Top middle

form_coord(1, :) = [x_leader - (sqrt(3)/2)*spacing, y_leader + spacing/2  ]; % Middle left
    form_coord(2, :) = [x_leader - (sqrt(3)/2)*spacing , y_leader - spacing/2]; % Bottom middle
    form_coord(3, :) = [x_leader + (sqrt(3)/2)*spacing, y_leader - spacing/2 ]; % Middle right
    form_coord(4, :) = [x_leader + (sqrt(3)/2)*spacing , y_leader + spacing/2]; % Top middle


    % Calculate deltas
    for i = 1:4
        deltas(i, 1) = form_coord(i, 1) - x_leader;
        deltas(i, 2) = form_coord(i, 2) - y_leader;
    end
    
    delta_x = deltas(:, 1);
    delta_y = deltas(:, 2);
end


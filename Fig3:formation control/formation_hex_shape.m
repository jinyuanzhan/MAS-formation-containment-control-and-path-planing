function [form_coord, delta_x, delta_y] = formation_hex_shape(x_leader, y_leader, spacing)


    % Initialize coordinates for 6 followers
    form_coord = zeros(6, 2); 
    deltas = zeros(6, 2); 

    % Define the 'U' shape relative positions from the leader
    % Assuming the leader is at the bottom center of the 'U'
    form_coord(1, :) = [x_leader , y_leader + spacing]; % top left
    form_coord(2, :) = [x_leader - (sqrt(3)/2)*spacing, y_leader + spacing/2];   % middle left

    form_coord(3, :) = [x_leader - (sqrt(3)/2)*spacing , y_leader - spacing/2]; 


    % top middle left
    form_coord(4, :) = [x_leader  , y_leader - spacing ]; 
    form_coord(5, :) = [x_leader + (sqrt(3)/2)*spacing, y_leader - spacing/2];             % bottom middle

    form_coord(6, :) = [x_leader + (sqrt(3)/2)*spacing ,y_leader + spacing/2];   % middle right


%form_coord(21, :) = [x_leader , y_leader+ 2 ];

 %----


    % Calculate deltas
    for i = 1:6
        deltas(i, 1) = form_coord(i, 1) - x_leader;
        deltas(i, 2) = form_coord(i, 2) - y_leader;
    end
    
    delta_x = deltas(:, 1);
    delta_y = deltas(:, 2);
end
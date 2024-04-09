function [form_coord, delta_x, delta_y] = formation_heart_shape(x_leader, y_leader, spacing)


    

    % Initialize coordinates for 6 followers
    form_coord = zeros(10, 2); 
    deltas = zeros(10, 2); 

    % Define the 'U' shape relative positions from the leader
    % Assuming the leader is at the bottom center of the 'U'
    form_coord(1, :) = [x_leader  , y_leader + 20]; % top left
    form_coord(2, :) = [x_leader - 60, y_leader + 60];   % middle left

    form_coord(3, :) = [x_leader - 110 , y_leader + 50]; 


    % top middle left
    form_coord(4, :) = [x_leader - 150 , y_leader + 5 ]; 
    form_coord(5, :) = [x_leader - 110, y_leader  - 40 ];             % bottom middle

    form_coord(6, :) = [x_leader , y_leader - 90];   % middle right

    form_coord(7, :) = [x_leader + 110 , y_leader  - 40]; % top right
%-----

    form_coord(8, :) = [x_leader + 150, y_leader + 5]; % top right
    form_coord(9, :) = [x_leader + 110, y_leader + 50]; % top right
     form_coord(10, :) = [x_leader + 60, y_leader + 60]; % top right
    

%form_coord(21, :) = [x_leader , y_leader+ 2 ];

 %----


    % Calculate deltas
    for i = 1:10
        deltas(i, 1) = form_coord(i, 1) - x_leader;
        deltas(i, 2) = form_coord(i, 2) - y_leader;
    end
    
    delta_x = deltas(:, 1);
    delta_y = deltas(:, 2);
end
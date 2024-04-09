function [form_coord, delta_x, delta_y] = formation_triangle_shape(x_leader, y_leader, spacing)
    % Initialize coordinates for 3 followers in a triangle formation
    form_coord = zeros(3, 2); 
    deltas = zeros(3, 2); 
    
    % The first follower is directly above the leader
    form_coord(1, :) = [x_leader, y_leader + spacing]; 
    
    % Calculate the positions of the other two followers to form an equilateral triangle
    % These followers are positioned at 60 degrees to the left and right of the vertical follower
    % Using 30-60-90 triangle ratios, the horizontal distance from the leader is spacing*sin(60 degrees)
    % The vertical distance from the leader is spacing*cos(60 degrees), but since these agents are below the leader, it's subtracted
    form_coord(2, :) = [x_leader - spacing * sin(pi / 3), y_leader - spacing * cos(pi / 3)]; % left follower
    form_coord(3, :) = [x_leader + spacing * sin(pi / 3), y_leader - spacing * cos(pi / 3)]; % right follower

    % Calculate deltas for each follower relative to the leader
    for i = 1:3
        deltas(i, 1) = form_coord(i, 1) - x_leader; % Delta X
        deltas(i, 2) = form_coord(i, 2) - y_leader; % Delta Y
    end
    
    delta_x = deltas(:, 1);
    delta_y = deltas(:, 2);
end


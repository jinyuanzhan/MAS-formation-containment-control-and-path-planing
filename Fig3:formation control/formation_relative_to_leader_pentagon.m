

function [form_coord, delta_x, delta_y] = formation_relative_to_leader_pentagon(x_leader, y_leader, r)
    form_coord = zeros(5, 2); 
    deltas = zeros(5, 2); 

    
    for i = 1:5
        angle_deg = (i - 1) * 72; 
        angle_rad = deg2rad(angle_deg);

        
        offsetX = r * cos(angle_rad);
        offsetY = r * sin(angle_rad);

      
        form_coord(i, :) = [x_leader + offsetX, y_leader + offsetY];

    
        deltas(i, :) = [offsetX, offsetY];
    end

 
    delta_x = deltas(:, 1);
    delta_y = deltas(:, 2);
end



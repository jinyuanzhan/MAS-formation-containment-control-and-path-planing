
function [form_coords, delta_xs, delta_ys] = formation_relative_to_leader_pentagon_two_lost(x_leaders, y_leaders, r)
    num_leaders = size(x_leaders, 1); 
    num_vertices = 5; 
    total_vertices = num_leaders * num_vertices; 
    form_coords = zeros(total_vertices, 2); 
    delta_xs = zeros(total_vertices, 1); 
    delta_ys = zeros(total_vertices, 1); 
    for j = 1:num_leaders
        x_leader = x_leaders(j);
        y_leader = y_leaders(j);
        base_index = (j - 1) * num_vertices; 

        
        for i = 1:num_vertices
            angle_deg = (i - 1) * 72; 
            angle_rad = deg2rad(angle_deg); 

            offsetX = r * cos(angle_rad); 
            offsetY = r * sin(angle_rad); 
            index = base_index + i; 
            form_coords(index, :) = [x_leader + offsetX, y_leader + offsetY]; 
            delta_xs(index) = offsetX; 
            delta_ys(index) = offsetY; 
        end
    end
end



function [form_coord, delta_x, delta_y] = formation_relative_to_leader_pentagon(x_leader, y_leader, r)
    form_coord = zeros(5, 2); % 初始化五边形五个顶点的坐标
    deltas = zeros(5, 2); % 初始化delta值

    for i = 1:5
        angle_deg = (i - 1) * 72;
        angle_rad = deg2rad(angle_deg); 

     
        offsetX = r * cos(angle_rad);
        offsetY = r * sin(angle_rad);

   
        form_coord(i, :) = [x_leader + offsetX, y_leader + offsetY];

    
        deltas(i, :) = [offsetX, offsetY];
    end

    % 提取delta_x和delta_y
    delta_x = deltas(:, 1);
    delta_y = deltas(:, 2);
end

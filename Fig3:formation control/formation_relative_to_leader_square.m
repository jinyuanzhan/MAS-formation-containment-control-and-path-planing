function [form_coord, delta_x, delta_y] = formation_relative_to_leader_square(x_leader, y_leader, r)
    form_coord = zeros(4, 2); % 初始化正方形四个顶点的坐标
    deltas = zeros(4, 2); % 初始化delta值

    % 正方形的四个顶点相对于中心点的偏移量
    offsets = [-r, r; r, r; r, -r; -r, -r];
    
    for i = 1:4
        % 计算每个顶点的坐标
        form_coord(i, :) = [x_leader + offsets(i, 1), y_leader + offsets(i, 2)];

        % 计算delta值
        deltas(i, :) = [offsets(i, 1), offsets(i, 2)];
    end
    
    % 提取delta_x和delta_y
    delta_x = deltas(:, 1);
    delta_y = deltas(:, 2);
end

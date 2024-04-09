
function [form_coords, delta_xs, delta_ys] = formation_relative_to_leader_pentagon_two_lost(x_leaders, y_leaders, r)
    num_leaders = size(x_leaders, 1); % 领导者的数量
    num_vertices = 5; % 五边形的顶点数
    total_vertices = num_leaders * num_vertices; % 总顶点数
    form_coords = zeros(total_vertices, 2); % 初始化二维数组来存储所有领导者的队形坐标
    delta_xs = zeros(total_vertices, 1); % 初始化delta_x
    delta_ys = zeros(total_vertices, 1); % 初始化delta_y

    for j = 1:num_leaders
        x_leader = x_leaders(j);
        y_leader = y_leaders(j);
        base_index = (j - 1) * num_vertices; % 计算当前领导者的基础索引

        % 计算对于当前领导者的五边形队形
        for i = 1:num_vertices
            angle_deg = (i - 1) * 72; % 五边形每个顶点的角度
            angle_rad = deg2rad(angle_deg); % 将角度转换为弧度

            offsetX = r * cos(angle_rad); % 计算偏移量
            offsetY = r * sin(angle_rad); % 计算偏移量

            index = base_index + i; % 计算当前顶点在数组中的索引
            form_coords(index, :) = [x_leader + offsetX, y_leader + offsetY]; % 为当前领导者计算队形坐标
            delta_xs(index) = offsetX; % 记录delta_x
            delta_ys(index) = offsetY; % 记录delta_y
        end
    end
end

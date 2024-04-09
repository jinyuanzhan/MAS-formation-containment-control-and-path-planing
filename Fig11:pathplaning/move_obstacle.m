function [obst_pos, obstacle_struct] = move_obstacle(obst_pos, num_obstacles, obstacle_struct, ~)
    k = 5; % 指定只移动第五个障碍物
    if k <= num_obstacles % 确保障碍物索引不超出范围
        % 更新边框的位置
        obstacle_struct.o(k).XData = obstacle_struct.o(k).XData + 1;
        obstacle_struct.o(k).YData = obstacle_struct.o(k).YData - 0;

        % 更新填充颜色的位置
        % 假设fill_handles就是您之前保存在obstacle_struct.o1中的fill对象的句柄
        fill_handle = obstacle_struct.o1(k); % 获取对应的fill对象句柄
        % 对于fill对象，我们需要获取当前的顶点坐标，然后统一移动这些坐标
        x_vertices = get(fill_handle, 'XData') + 1;
        y_vertices = get(fill_handle, 'YData') - 0;
        set(fill_handle, 'XData', x_vertices, 'YData', y_vertices);
        
        % 重新计算第五个障碍物中心点的位置
        % 这里我们取边框的新位置来更新障碍物的中心位置
        obst_pos(k, 1) = mean(obstacle_struct.o(k).XData);
        obst_pos(k, 2) = mean(obstacle_struct.o(k).YData);
    end
end

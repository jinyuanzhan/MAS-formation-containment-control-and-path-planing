

function [obst_pos, obstacle_struct] = move_obstacle(obst_pos, num_obstacles, obstacle_struct, ~)
    k = 5; % 指定只移动第五个障碍物
    if k <= num_obstacles % 确保障碍物索引不超出范围
        % 平移第五个障碍物的所有点
        obstacle_struct.o(k).XData = obstacle_struct.o(k).XData - 0.35;
        obstacle_struct.o(k).YData = obstacle_struct.o(k).YData - 0.2;
        % 
        % 更新第五个障碍物中心点的位置（假设您有某种方式定义中心点）
        obst_pos(k, 1) = obst_pos(k, 1) - 0.25;
        obst_pos(k, 2) = obst_pos(k, 2) - 0.2;
    end
end

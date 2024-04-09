
% %This code randomly generates obstacles of a certain radius at a random
% %location
% 
% function [x_obs, y_obs, r_obs] = obstacles_initial(num_obstacles)
% 
% 
%     % fixed  obs
%     % obstacles = [
%     %     -20, 200, 20; 
%     %     420, 220, 40; 
%     %     400, 70, 20;
%     %     290, 140,30;
%     %     160, 195, 20;
%     % ];
% 
%     % obstacles = [
%     %     -50, 155, 5; 
%     %     420, 300, 40; 
%     %     400, 300, 20;
%     %     290, 300,30;
%     %     550, 125, 10;
%     % ];
%     %  obstacles = [
%     %     -50, 75, 5; 
%     %     150, 110, 5; 
%     %     290, 90, 5;
%     %     400, 30,5;
%     %     300, 66, 5;
%     % ];
% 
%     obstacles = [
%         -1050, 265, 5; 
%         1010, 250, 5; 
%         1000, 230, 5;
%         1090, 300,30;
%         1050, 125, 10;
%     ];
% 
% 
% 
% 
%     % 
%     x_obs = obstacles(:, 1)';
%     y_obs = obstacles(:, 2)';
%     r_obs = obstacles(:, 3)';
% 
% 
%     % x_obs = randi([-20 180], 1, num_obstacles);
%     % y_obs = randi([-20 200], 1, num_obstacles);
%     % r_obs = randi([7 12], 1, num_obstacles); 
% 
%     % % 检查确保障碍物之间不相交
%     % for i = 1:num_obstacles
%     %     for k = 1:num_obstacles
%     %         if k ~= i
%     %             % 检查障碍物之间是否相交
%     %             while sqrt((x_obs(k)-x_obs(i))^2 + (y_obs(k)-y_obs(i))^2) < (r_obs(k) + r_obs(i) + 15)
%     %                 x_obs(k) = randi([-20 180]);
%     %                 y_obs(k) = randi([-20 200]);
%     %                 r_obs(k) = randi([7 12]);
%     %             end
%     %         end
%     %     end
%     % end
% end

function [x_obs, y_obs, r_obs] = obstacles_initial(num_obstacles)
    % 设置障碍物的x和y坐标的范围
    x_min = -250;
    x_max = 800;
    y_min =  10%130;
    y_max = 400%280;
    r = 3; % 所有障碍物的半径
    min_distance = 25 + 2*r; % 最小间隔加上障碍物直径

    % 初始化障碍物坐标和半径的数组
    x_obs = zeros(1, num_obstacles);
    y_obs = zeros(1, num_obstacles);
    r_obs = r * ones(1, num_obstacles); % 创建一个所有元素都是r的数组

    for i = 1:num_obstacles
        valid_position = false;
        while ~valid_position
            x_temp = (x_max - x_min) .* rand() + x_min;
            y_temp = (y_max - y_min) .* rand() + y_min;

            % 检查与已有障碍物的距离
            if i == 1
                valid_position = true; % 第一个障碍物直接放置
            else
                distances = sqrt((x_obs(1:i-1) - x_temp).^2 + (y_obs(1:i-1) - y_temp).^2);
                if all(distances >= min_distance)
                    valid_position = true;
                end
            end
        end
        % 保存有效位置
        x_obs(i) = x_temp;
        y_obs(i) = y_temp;
    end
end

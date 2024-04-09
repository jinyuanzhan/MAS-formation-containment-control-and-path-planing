
%This code randomly generates obstacles of a certain radius at a random
%location

function [x_obs, y_obs, r_obs] = obstacles_initial(num_obstacles)
    

    % fixed  obs
    % obstacles = [
    %     -20, 200, 20; 
    %     420, 220, 40; 
    %     400, 70, 20;
    %     290, 140,30;
    %     160, 195, 20;
    % ];

    % obstacles = [
    %     -50, 155, 5; 
    %     420, 300, 40; 
    %     400, 300, 20;
    %     290, 300,30;
    %     550, 125, 10;
    % ];
     obstacles = [
        -1000, 200, 100; 
        2000, 72, 40; 
        2000, 260, 5;
        2000, 150, 5;
        2050, 125, 10;
    ];

    % 
    x_obs = obstacles(:, 1)';
    y_obs = obstacles(:, 2)';
    r_obs = obstacles(:, 3)';

    
    % x_obs = randi([-20 180], 1, num_obstacles);
    % y_obs = randi([-20 200], 1, num_obstacles);
    % r_obs = randi([7 12], 1, num_obstacles); 

    % % 检查确保障碍物之间不相交
    % for i = 1:num_obstacles
    %     for k = 1:num_obstacles
    %         if k ~= i
    %             % 检查障碍物之间是否相交
    %             while sqrt((x_obs(k)-x_obs(i))^2 + (y_obs(k)-y_obs(i))^2) < (r_obs(k) + r_obs(i) + 15)
    %                 x_obs(k) = randi([-20 180]);
    %                 y_obs(k) = randi([-20 200]);
    %                 r_obs(k) = randi([7 12]);
    %             end
    %         end
    %     end
    % end
end

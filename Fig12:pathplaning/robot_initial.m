%This code randomly generates robots of a radius 3 at a random pos

function [x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_robots)

    % followe 位置和半径初始化
    rng(2)
    
    x = randi([-145 -100], 1, num_robots-1);
    y = randi([165 210], 1, num_robots-1);
    r = 3 * ones(1, num_robots-1);


    % % leader 初始化 
    % x_leader = -150;   %randi([10 190]);
    % y_leader = 0;    %randi([-180 -160]);
    % r_leader = 6;      % 

    
    % leader的初始化
    x_leader = [-150; 180;150;-180;]; % 两个leader的x坐标
    y_leader = [0; 340;0;340];   % 两个leader的y坐标
    r_leader = [6; 6;6;6];       % 两个leader的半径

 % x_leader = [-100; 200]; % 两个leader的x坐标
 %    y_leader = [200; 200];   % 两个leader的y坐标
 %    r_leader = [5; 5];   



    % % 第二个leader
    % x_leader_2 = -110;
    % y_leader_2 = 125;
    % r_leader_2 = 3;
    % 
    % x_leader = [x_leader;x_leader_2];
    % y_leader = [ y_leader; y_leader_2];
    % r_leader = [r_leader;r_leader_2];
    % 

    %start_point = [-210, 200];
    %end_point = [380, 200];

    % for i = 1:num_robots-1
    %     for k = 1:num_robots-1
    %         if k ~= i
    %             % 检查追随者之间是否相交
    %             while sqrt((x(k)-x(i))^2 + (y(k)-y(i))^2) < (r(k) + r(i) + 10)
    %                 x(k) = randi([-145 -100]);
    %                 y(k) = randi([165 210]);
    %             end
    %         end
    %     end
    %     % 检查领导者是否与当前追随者相交
    %     while sqrt((x_leader-x(i))^2 + (y_leader-y(i))^2) < (r_leader + r(i) + 10)
    %         x(i) = randi([-145 -100]);
    %         y(i) = randi([165 210]);
    %     end
    % end

    % % 排序机器人的位置
    % % 创建一个包含 x, y 坐标的临时矩阵
    % coords = [x; y];
    % 
    % % 按 y 坐标降序排序，如果 y 相同，则按 x 坐标降序排序
    % [~, sortedIndices] = sortrows(coords', [-2, -1]);
    % 
    % % 应用排序
    % x = x(sortedIndices);
    % y = y(sortedIndices);

     % x = [-280,-256,-256,-227];
     % y = [234,2-5,189,165];

end


% 
% 
% function [x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_robots)
% 
%     % follower位置和半径初始化
%     rng(1); % 设置随机数生成器的种子，以便结果可复现
%     x = randi([-230 -190], 1, num_robots-2);
%     y = randi([180 240], 1, num_robots-2);
%     r = 3 * ones(1, num_robots-1);
% 
%     % leader初始化 
%     x_leader = [-110; -110]; % 两个leader的x坐标
%     y_leader = [200; 125];   % 两个leader的y坐标
%     r_leader = [3; 3];       % 两个leader的半径
% 
%     %start_point = [-210, 200];
%     %end_point = [380, 200];
% 
%     for i = 1:num_robots-2
%         for j = 1:size(x_leader, 1) % 对每个leader
%             % 检查领导者是否与当前追随者相交
%             while sqrt((x_leader(j) - x(i))^2 + (y_leader(j) - y(i))^2) < (r_leader(j) + r(i) + 10)
%                 x(i) = randi([-260 -180]);
%                 y(i) = randi([130 270]);
%             end
%         end
% 
%         for k = 1:num_robots-2
%             if k ~= i
%                 % 检查追随者之间是否相交
%                 while sqrt((x(k) - x(i))^2 + (y(k) - y(i))^2) < (r(k) + r(i) + 10)
%                     x(k) = randi([-260 -180]);
%                     y(k) = randi([130 270]);
%                 end
%             end
%         end
%     end
% 
%     % ... 其余代码不变 ...
% end

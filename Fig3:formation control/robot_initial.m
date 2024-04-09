%This code randomly generates robots of a radius 3 at a random pos

function [x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_robots)

    % followe 位置和半径初始化
    rng(3)
    
    x = randi([-185 -100], 1, num_robots-1);
    y = randi([165 290], 1, num_robots-1);
    r = 3 * ones(1, num_robots-1);


    % leader 初始化 
    x_leader = -110;   %randi([10 190]);
    y_leader = 200;    %randi([-180 -160]);
    r_leader = 3;      % 


    for i = 1:num_robots-1
        for k = 1:num_robots-1
            if k ~= i
              
                while sqrt((x(k)-x(i))^2 + (y(k)-y(i))^2) < (r(k) + r(i) + 10)
                    x(k) = randi([-145 -100]);
                    y(k) = randi([165 210]);
                end
            end
        end
      
        while sqrt((x_leader-x(i))^2 + (y_leader-y(i))^2) < (r_leader + r(i) + 10)
            x(i) = randi([-145 -100]);
            y(i) = randi([165 210]);
        end
    end

   

end




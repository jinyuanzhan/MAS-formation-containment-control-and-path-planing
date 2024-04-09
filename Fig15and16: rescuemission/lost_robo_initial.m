function [x_lost,y_lost,r] = lost_robo_initial(num_lost)
    
    % followe 位置和半径初始化
    rng(1)
    % x_lost = randi([400 500], 1, num_lost);
    % y_lost = randi([100 400], 1, num_lost);

   x_lost = [478,390,410];
   y_lost = [245 ,80,150];



    r = num_lost * ones(1, num_lost);

    for i = 1:num_lost
        for k = 1:num_lost
            if k ~= i
                % 检查追随者之间是否相交
                while sqrt((x_lost(k)-x_lost(i))^2 + (y_lost(k)-y_lost(i))^2) < (r(k) + r(i) + 10)
                    x_lost(k) = randi([-260 -180]);
                    y_lost(k) = randi([130 270]);
                end
            end
        end
    end
  
end
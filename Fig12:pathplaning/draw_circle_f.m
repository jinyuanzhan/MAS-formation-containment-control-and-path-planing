


function [x_cell,y_cell] = draw_circle_f(x,y,r)
    %Draw a circle with 100 points given the circle's x,y location and 
    %radius. This code also takes x,y,r as vectors.
    theta = linspace(0,2*pi,100);
    x_cell = {};
    y_cell = {};
    for i = 1:size(x,1)
        x_cell{i} = r(i) * cos(theta) + x(i);
        y_cell{i} = r(i) * sin(theta) + y(i);
    end
end

% 
% fill(x, y, 'k'); % 使用黑色填充圆形 ('k' 代表黑色)
%         hold on; % 保持当前图像，以便在同一图形上绘制多个圆形
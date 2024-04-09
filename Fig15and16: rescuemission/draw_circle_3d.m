function [X, Y, Z] = draw_circle_3d(x, y, z, r)
    % 绘制一个三维半球体，半球体位于给定的x,y,z位置，具有给定的半径r。
    % 此代码还接受向量形式的x,y,z,r，以便同时绘制多个圆形。
    
    % 生成theta和phi向量，但是用更少的点来使网格看起来更稀疏
    theta = linspace(0, 2 * pi, 15); % 减少点数以使网格线更稀疏
    phi = linspace(0, pi/2, 7); % 同样减少点数
   
    [Theta, Phi] = meshgrid(theta, phi);
    
    % 计算三维半球体的X,Y,Z坐标
    X = r * sin(Phi) .* cos(Theta) + x;
    Y = r * sin(Phi) .* sin(Theta) + y;
    z_factor = 0.1; % 指定一个小因子以减少半球体的高度
    Z = z + r * cos(Phi) * z_factor; % 修改Z坐标以调整半球体的高度
end

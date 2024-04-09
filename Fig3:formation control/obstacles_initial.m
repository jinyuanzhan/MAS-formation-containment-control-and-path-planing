
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
        -10, 200, 100; 
        2000, 72, 40; 
        2000, 260, 5;
        2000, 150, 5;
        2050, 125, 10;
    ];

    % 
    x_obs = obstacles(:, 1)';
    y_obs = obstacles(:, 2)';
    r_obs = obstacles(:, 3)';

    
end

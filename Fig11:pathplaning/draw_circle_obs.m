function [x_cell, y_cell, fill_handles] = draw_circle_obs(x, y, r)
    %Draw and fill circles given the circle's x, y location and radius.
    %This code also takes x, y, r as vectors. It fills the circles with color
    %and returns the vertices along with fill object handles.

    % Define the number of points on the circle
    num_points = 100;

    % Define the color for filling the circle
    color = [160, 160, 160] / 255;  % Gray color

    % Preallocate cell arrays for performance
    x_cell = cell(size(x, 1), 1);
    y_cell = cell(size(y, 1), 1);
    fill_handles = gobjects(size(x, 1), 1);  % Array to store fill handles

    % Generate points for each circle
    theta = linspace(0, 2*pi, num_points);
    
    for i = 1:size(x, 1)
        % Calculate the x and y coordinates for the circle points
        x_points = r(i) * cos(theta) + x(i);
        y_points = r(i) * sin(theta) + y(i);
        
        % Store the circle points in the cell arrays
        x_cell{i} = x_points;
        y_cell{i} = y_points;
        
        % Fill the circle with the specified color and store the fill handle
        fill_handles(i) = fill(x_points, y_points, color);
        hold on; % Hold on to plot multiple circles
    end
    
    hold off; % Release the plot after all circles are drawn
end

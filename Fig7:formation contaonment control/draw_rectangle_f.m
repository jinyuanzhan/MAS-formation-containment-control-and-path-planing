
function [x_rect, y_rect] = draw_rectangle_f(x, y, w, h, height_factor)
    % Draw and fill rectangles given the center's x,y location, width, and height.
    % This code also takes x, y, w, h as vectors, and height_factor to increase height.
    % It fills the rectangles with color and returns the vertices.

    x_rect = cell(size(x,1), 1);
    y_rect = cell(size(y,1), 1);

    for i = 1:size(x,1)
        % Adjust the height by height_factor
        adjusted_h = h(i) * height_factor;

        % Calculate the vertices of the rectangle with the adjusted height
        x_top_left = x(i) - w(i)/2;
        y_top_left = y(i) + (adjusted_h/2);
        
        x_top_right = x(i) + w(i)/2;
        y_top_right = y(i) + (adjusted_h/2);
        
        x_bottom_right = x(i) + w(i)/2;
        y_bottom_right = y(i) - adjusted_h/2;
        
        x_bottom_left = x(i) - w(i)/2;
        y_bottom_left = y(i) - adjusted_h/2;
        
        % Store the vertices in clockwise order starting from the top left
        x_vertices = [x_top_left, x_top_right, x_bottom_right, x_bottom_left, x_top_left];
        y_vertices = [y_top_left, y_top_right, y_bottom_right, y_bottom_left, y_top_left];
        
        % Assign the vertices to the output cells
        x_rect{i} = x_vertices;
        y_rect{i} = y_vertices;


       
color = [160, 160, 160] / 255;


fill(x_vertices, y_vertices, color);

        % Fill the rectangle with color
       
        hold on; % Hold on to plot multiple rectangles
    end

    hold off; % Release the plot after all rectangles are drawn
end

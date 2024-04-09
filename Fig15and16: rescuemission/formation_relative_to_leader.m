%  %This code caluclates the formation center and the robots goal
%     %locations in the formation


function [form_coord,delta_x,delta_y] = formation_relative_to_leader(x_leader, y_leader, r)
    remainingVertices = zeros(4, 2); 
    deltas = zeros(4, 2); 

    % Reference point from known vertex
    referenceX = x_leader- r*0.85065;
    referenceY = y_leader;

    % Convert side length to radius of circumscribed circle
    R = r / (2 * sin(pi / 5));

    angle = 0;

    for i = 1:4
       
        angle = angle + 72;
        newX = referenceX + R * cosd(angle);
        newY = referenceY + R * sind(angle);
        remainingVertices(i, :) = [newX, newY];

        % Calculate delta x and delta y
        deltas(i, 1) = newX - referenceX- r * 0.85065;
        deltas(i, 2) = newY - referenceY;
        delta_x = deltas(:,1);
        delta_y = deltas(:,2);
        form_coord = remainingVertices;
    end
end

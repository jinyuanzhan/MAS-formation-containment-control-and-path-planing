function [form_coord, delta_x, delta_y] = formation_u_shape(x_leader, y_leader, spacing)
    % Initialize coordinates for 6 followers
    form_coord = zeros(21, 2); 
    deltas = zeros(21, 2); 

    % Define the 'U' shape relative positions from the leader
    % Assuming the leader is at the bottom center of the 'U'
    form_coord(1, :) = [x_leader - 90, y_leader + 150]; % top left
    form_coord(2, :) = [x_leader - 90, y_leader + 90];   % middle left

    form_coord(3, :) = [x_leader - 60 , y_leader + 32];           % top middle left
    form_coord(4, :) = [x_leader , y_leader ]; 
    form_coord(5, :) = [x_leader + 60, y_leader + 32 ];             % bottom middle

    form_coord(6, :) = [x_leader + 90, y_leader + 90];   % middle right
    form_coord(7, :) = [x_leader + 90, y_leader + 150]; % top right
%-----

    form_coord(8, :) = [x_leader + 330, y_leader + 150]; % top right
    form_coord(9, :) = [x_leader + 250, y_leader + 150]; % top right
    form_coord(10, :) = [x_leader + 190, y_leader + 125]; % top right

    form_coord(11, :) = [x_leader + 180, y_leader + 75]; % top right
    form_coord(12, :) = [x_leader + 190, y_leader + 26 ]; % top right
    form_coord(13, :) = [x_leader + 250, y_leader ]; % top right
    form_coord(14, :) = [x_leader + 330, y_leader ]; % top right
%------

form_coord(15, :) = [x_leader + 430, y_leader+150 ]; % top right
form_coord(16, :) = [x_leader + 430, y_leader+100 ]; % top right
form_coord(17, :) = [x_leader + 430, y_leader+ 50]; % top right
form_coord(18, :) = [x_leader + 430, y_leader ]; % top right
form_coord(19, :) = [x_leader + 480, y_leader ]; % top right
form_coord(20, :) = [x_leader + 530, y_leader ]; % top right
form_coord(21, :) = [x_leader + 580, y_leader ]; % top right

%form_coord(21, :) = [x_leader , y_leader+ 2 ];

 %----


    % Calculate deltas
    for i = 1:21
        deltas(i, 1) = form_coord(i, 1) - x_leader;
        deltas(i, 2) = form_coord(i, 2) - y_leader;
    end
    
    delta_x = deltas(:, 1);
    delta_y = deltas(:, 2);
end

%----------

% 
% 
% 
% 
% function [form_coord, delta_x, delta_y] = formation_u_shape(x_leader, y_leader, spacing)
%     % Initialize coordinates for 6 followers
%     form_coord = zeros(20, 2); 
%     deltas = zeros(20, 2); 
% 
%     % Define the 'U' shape relative positions from the leader
%     % Assuming the leader is at the bottom center of the 'U'
%     form_coord(1, :) = [x_leader - 30, y_leader + 55]; % top left
%     form_coord(2, :) = [x_leader - 30, y_leader + 32];   % middle left
% 
%     form_coord(3, :) = [x_leader - 20 , y_leader + 12];           % top middle left
%     form_coord(4, :) = [x_leader + 20, y_leader + 12 ];             % bottom middle
% 
%     form_coord(5, :) = [x_leader + 30, y_leader + 32];   % middle right
%     form_coord(6, :) = [x_leader + 30, y_leader + 55]; % top right
% %-----
% 
%     form_coord(7, :) = [x_leader + 89, y_leader + 48]; % top right
%     form_coord(8, :) = [x_leader + 78, y_leader + 25]; % top right
%     form_coord(9, :) = [x_leader + 89, y_leader + 6]; % top right
% 
%     form_coord(10, :) = [x_leader + 108, y_leader + 55]; % top right
%     form_coord(11, :) = [x_leader + 108, y_leader ]; % top right
%     form_coord(12, :) = [x_leader + 132.5, y_leader + 55]; % top right
%     form_coord(13, :) = [x_leader + 132.5, y_leader ]; % top right
% %------
% 
% form_coord(14, :) = [x_leader + 188, y_leader+55 ]; % top right
% form_coord(15, :) = [x_leader + 188, y_leader+37 ]; % top right
% form_coord(16, :) = [x_leader + 188, y_leader+18 ]; % top right
% form_coord(17, :) = [x_leader + 188, y_leader ]; % top right
% form_coord(18, :) = [x_leader + 211, y_leader ]; % top right
% form_coord(19, :) = [x_leader + 226, y_leader ]; % top right
% form_coord(20, :) = [x_leader + 241, y_leader ]; % top right
% 
% %----
% 
% 
% 
% 
% 
%     % Calculate deltas
%     for i = 1:9
%         deltas(i, 1) = form_coord(i, 1) - x_leader;
%         deltas(i, 2) = form_coord(i, 2) - y_leader;
%     end
% 
%     delta_x = deltas(:, 1);
%     delta_y = deltas(:, 2);
% end
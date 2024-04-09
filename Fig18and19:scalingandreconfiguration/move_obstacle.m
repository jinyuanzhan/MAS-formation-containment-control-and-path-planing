function [obst_pos, obstacle_struct] = move_obstacle(obst_pos, num_obstacles, obstacle_struct, ~)
    k = 5; 
    if k <= num_obstacles 
        obstacle_struct.o(k).XData = obstacle_struct.o(k).XData - 0.2;
        obstacle_struct.o(k).YData = obstacle_struct.o(k).YData - 0.2;
       
        obst_pos(k, 1) = mean(obstacle_struct.o(k).XData); %- r_ob * cos(linspace(0, 2 * pi, 100));
        obst_pos(k, 2) = mean(obstacle_struct.o(k).YData); %- r_ob * sin(linspace(0, 2 * pi, 100));
    end
end

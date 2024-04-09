clear;
close all;
clc;

% v = VideoWriter('Circile_trajectory.mp4', 'MPEG-4');
% v.FrameRate = 24; %
% open(v);

num_agents = 7;
num_obstacles = 5;
[x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_agents);
[x_obs, y_obs, r_obs] = obstacles_initial (num_obstacles);
K_controller_formation = 1;
d_max_repulsion_obs = 9;    
d_max_repulsion_agent = 9; 
k_repulsion_obs=0;               
k_repulsion_agent = 0;
safe_dis_agent = 2;
e_form = 0.05;
repulsion_max_obs = 1; 
repulsion_max_agent = 1;
zeta_agent = 0.0003;
side_length =30;
 steps = 1;
actual_time_step = 0.06; 
yita=0.002;
[target_position, delta_agent_x , delta_agent_y] = formation_hex_shape(x_leader,y_leader,side_length);

delta_xy= [delta_agent_x, delta_agent_y];

target_position_tem = target_position;

adj_matrix=[0 1 1 0 0 ;                 
            0 0 0 0 2 ;
            0 0 0 2 0 ;
            0 0 0 0 1 ;
            0 0 0 0 0 ;];


follower_pos =[x' , y'];
leader_pos  =[x_leader, y_leader];

all_agent = [leader_pos;follower_pos];

obst_pos    = [x_obs', y_obs'];
end_pos    =[600,200];


%% Plot robots, obstacles, goal

[x1,y1] = draw_circle_f(follower_pos(:,1),follower_pos(:,2),r );
 [xl,yl] = draw_circle_f(leader_pos(:,1),leader_pos(:,2),r_leader);
 
 [xob,yob] = draw_circle_f(obst_pos(:,1),obst_pos(:,2),r_obs);
 
[x_end,y_end] = draw_circle_f(end_pos(1),end_pos(2),3);    

[x_tar,y_tar] = draw_circle_f(target_position(:,1),target_position(:,2),r);

%stracture
map.agentx = x1;
map.agenty = y1;
map.leaderx = xl;
map.leadery = yl;
map.obstaclex = xob;
map.obstacley = yob;
map.endx = x_end;
map.endy = y_end; 
%
map.tarx = x_tar;
map.tary = y_tar;


 markers = {'o'};
 
 colors = {[255/255, 0/255, 0/255], [0/255, 0/255, 255/255], [0/255, 0/255, 255/255], ...
     [0/255, 0/255, 255/255],[0/255, 0/255, 255/255],[0/255, 0/255, 255/255],[0/255, 0/255, 255/255]};
% 
% for j = 1:length(follower_pos)
%     color = colors{mod(j-1, length(colors)) + 1};
%     plot_follower = plot(follower_pos(j,1), follower_pos(j,2), 'Marker', 'o', 'LineStyle', 'none', 'Color', color, 'MarkerSize', 10, 'LineWidth', 2);
%     hold on; % To plot all points without erasing the previous ones
% end


 
    for j = 1:num_agents - 1
        hold on
       % plot_end = plot(map.endx{1},map.endy{1},'g',LineWidth=2);
        plot_follower = plot(map.agentx{j},map.agenty{j},'b',LineWidth=2);
        robot_struct.rfollower(j) = plot_follower;
    end

    legend_handle_leader = plot_follower(1); 

    colorss = {[255/255, 0/255, 0/255], [35/255, 85/255, 206/255], [114/255, 47/255, 16/255], ...
     [30/255, 76/255, 99/255],[255/255, 165/255, 0/255],[114/255, 156/255, 68/255]};

    % target_plots = gobjects(num_agents - 1, 1); 
    % for i = 1:num_agents - 1
    %     hold on;
    %     target_plots(i) = plot(x_tar{i}, y_tar{i}, 'k-'); 
    % end

    for j = 1:1
        hold on
        plot_leader = plot(map.leaderx{j},map.leadery{j},'r',LineWidth=2);
        robot_struct.rleader(1) = plot_leader;
    end



    for j = 1:num_obstacles
        hold on
  
        plot_o = plot(map.obstaclex{j},map.obstacley{j},'r',LineWidth=0.5);
        obstacle_struct.o(j) = plot_o;
        hold off
    end





time_step = 0.1; %
time = (0:time_step:(steps-1)*time_step); 


title('Formation Control via Leader-Follower Strategy', 'FontSize', 17);  
xlabel('X position ', 'FontSize', 17);
ylabel('Y position ', 'FontSize', 17);


legend_labels = {'Leader', 'follower'};


legend_handles = gobjects(2 , 1); % +1 for the Obstacle

hold on; 
for i = 1:2
   
    legend_handles(i) = plot(NaN, NaN, 'MarkerEdgeColor', 'k', 'Color', colorss{i}, 'MarkerSize', 10,LineWidth=2);
end

hLegend = legend(legend_handles, legend_labels);


set(hLegend, 'FontSize', 12, 'Box', 'on');
hold off;



agent_trajectories(num_agents) = struct('x', [], 'y', []);

% sanpshot

 snapshot_steps = [1, 60, 120, 190, 250]; 

%%

xlim([-200, 200]);
ylim([0, 400]);  


set(gcf, 'Position',  [400, 400, 600, 600])
grid on;
box on;

%%

    steps = 1;
   
    F_rep_obs = zeros(num_obstacles,2);

    F_rep_aget = zeros(num_agents,2);

    form_goal = e_form*ones(num_agents-1,1)';

    f_goal = 1; 
    
   
    A = ((abs(norm(leader_pos)-norm(end_pos)) > f_goal)); %Check if formation is at goal
    
    B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) > form_goal); %Check if all robots are at their formation goal locations
    
  
    distance_threshold = 1.0;
 
    error_history = zeros(num_agents-1, 2800); 


    %%
groups = {[2:7,2]};
agent_lines = gobjects(sum(cellfun(@numel, groups)) - length(groups), 1); 

line_index = 1;

hold on

for g = 1:length(groups)
    group = groups{g};  
    for i = 1:length(group) - 1
        current_i = group(i);
        next_i = group(i + 1); 
       
        R = 205 / 255;
        G = 131 / 255;
        B = 135 / 255;

        agent_lines(line_index) = plot([all_agent(current_i,1), all_agent(next_i,1)], ...
                                       [all_agent(current_i,2), all_agent(next_i,2)], 'k--', 'LineWidth', 0.5,'HandleVisibility', 'off');

        line_index = line_index + 1;
    end
end
hold off;
    pause(1);
    %%

    trajectory = cell(num_agents, 1); %存储每个智能体的轨迹
for i = 1:num_agents
    trajectory{i} = [all_agent(i, :)];
end

delta_theta = 2 * pi / 470; 

x_center = -10;
y_center = 200;
radius = sqrt((x_leader - x_center)^2 + (y_leader - y_center)^2);
theta = atan2(y_leader - y_center, x_leader - x_center);


normalized_error_norm = zeros(num_agents-1, 2800);

    %%
    while A+B ~=0  

      


if steps > 0 && steps <= 480
    
    increment = (0.9 - 0.0001) / 480; 
    zeta_agent = zeta_agent + increment;
end

disp(steps)


        [target_position,~, ~] = formation_hex_shape(leader_pos(1),leader_pos(2),side_length);

     
    
     % Calculate the new leader position
      theta = theta + delta_theta ;
new_leader_pos_x = x_center + radius * cos(theta);
new_leader_pos_y = y_center + radius * sin(theta);

% Calculate velocity as the difference between the new position and the current position
velocity_leader_x = new_leader_pos_x - leader_pos(1);
velocity_leader_y = new_leader_pos_y - leader_pos(2);

velocity_leader =   [velocity_leader_x , velocity_leader_y];

% Update the current leader position using the velocity
leader_pos(1) = leader_pos(1) + velocity_leader_x;
leader_pos(2) = leader_pos(2) + velocity_leader_y;

    
       

       for i = 1:num_agents-1
           %If robots are not at their formation goal coordinates, move
            %them there
           if sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) > form_goal) ~= 0
           %if abs(vecnorm(follower_pos')-vecnorm(target_position')) > form_goal ~= 0

                a= abs(vecnorm(follower_pos(i)')-vecnorm(target_position(i)'));
                %disp(a)
                %For all obstacles in the map, calculate repulsive force
                %between robot and obstacle
                for k = 1:num_obstacles
                     F_rep_obs(k,:) = compute_repulsion_new(follower_pos(i,:),obst_pos(k,:),repulsion_max_obs,safe_dis_agent, k_repulsion_obs, d_max_repulsion_obs,r(i),r_obs(k));                                           
                end
                
                %For all other robots m in the map, calculate repulsive
                %force between robot m and robot i
                for m = 1:num_agents-1
                    if m == i %don't calculate a robots repulsive force against itself
                        continue
                    else
                        F_rep_aget(m,:) = compute_repulsion_new(follower_pos(i,:),follower_pos(m,:),repulsion_max_agent,safe_dis_agent,k_repulsion_agent, d_max_repulsion_agent,r(i),r(m));                                               
                       % function [repulsion] = compute_repulsion_new(robot_pose, obs_pose, repulsion_max, d_safe, k_rep, d_max,radius_agent,obs_radius)
                    end                
                end   

                % Sum of Repulsive Forces
                F_rep_obs_sum = sum(F_rep_obs, 1);
                F_rep_agent_sum = sum(F_rep_aget, 1);
               
               % Calculate attractive speed to push robot to formation
                % goal location according to the position difference
                %% formation control strategy (suppose leader only communicate with follower.
               position_error = (follower_pos(i,:) - target_position(i,:));
               % position_error = target_position - coord(i, :); 
                F_formation = (- zeta_agent) * position_error - yita *  sign(position_error);

                

                %%
                F_totx = (F_formation(1))+F_rep_agent_sum(1)+F_rep_obs_sum(1); %Add all forces in x
                
                F_toty = (F_formation(2))+F_rep_agent_sum(2)+F_rep_obs_sum(2); %Add all forces in y
               
                
                robot_struct.rfollower(i).XData = robot_struct.rfollower(i).XData+F_totx; %Update robot i's x direction by moving the circle
                robot_struct.rfollower(i).YData = robot_struct.rfollower(i).YData+F_toty;
                % pause(0.005);

                
              
                follower_pos(i, 1) = follower_pos(i, 1) + F_totx; % Update x-coordinate
                follower_pos(i, 2) = follower_pos(i, 2) + F_toty; % Update y-coordinate
                
           end

        current_error = norm(follower_pos(i,:) - target_position(i,:));
        error_history(i, steps) = current_error;
  
        normalized_error_norm(i, steps) = norm(error_history(i, steps)) / norm(error_history(i, 1));
    
    %         set(error_plot_handles(i), 'XData', 1:steps, 'YData', error_history(i, 1:steps));
    % 
    % drawnow; 
    % 
       end

      
         robot_struct.rleader(1).XData = robot_struct.rleader(1).XData+velocity_leader(1);
         robot_struct.rleader(1).YData = robot_struct.rleader(1).YData+velocity_leader(2);


 

    %%

        % 更新领导者的轨迹
        trajectory{1} = [trajectory{1}; leader_pos];
        
        % 更新跟随者的轨迹
        for i = 2:num_agents
            trajectory{i} = [trajectory{i}; follower_pos(i-1, :)];
        end


    %% 移动障碍物

    [obst_pos,obstacle_struct] = move_obstacle(obst_pos,5,obstacle_struct,r_obs(5));


    %%
hold on; 

if ismember(steps, snapshot_steps)
   
    for i = 1:num_agents
     
        agent_trajectories(i).x(end+1) = all_agent(i, 1);
        agent_trajectories(i).y(end+1) = all_agent(i, 2);
       
        marker = markers{mod(i-1, length(markers)) + 1};
        color = colors{mod(i-1, length(colors)) + 1};
        

    r_ne = 3; 
    

    rectangle('Position', [all_agent(i, 1)-r_ne, all_agent(i, 2)-r_ne, 2*r_ne, 2*r_ne], ...
             'Curvature', [1 1], 'EdgeColor', color, 'LineWidth', 1)
      
    end
   
    
    if steps > 0
    for g = 1:length(groups)
        group = groups{g};
        for i = 1:length(group) - 1
            current_i = group(i);
            next_i = group(i + 1);
           
        R = 0 / 255;
        G = 102 / 255;
        B = 0 / 255;
           line([all_agent(current_i,1), all_agent(next_i,1)], ...
                [all_agent(current_i,2), all_agent(next_i,2)], 'LineStyle', '--', 'Color', "k", 'LineWidth', 1,'HandleVisibility', 'off');
        end
    end
    end
end

hold off; % 结束绘制


%% 画线
 all_agent = [leader_pos; follower_pos];



  line_index = 1; 
for g = 1:length(groups)
    group = groups{g};
    for i = 1:length(group) - 1
        current_i = group(i);
        next_i = group(i + 1);
   
        if isgraphics(agent_lines(line_index))
           set(agent_lines(line_index), 'XData', [all_agent(current_i,1), all_agent(next_i,1)], ...
                                         'YData', [all_agent(current_i,2), all_agent(next_i,2)]);
        end
        line_index = line_index + 1;
    end

end
%%
% if steps > 360
%        
%         hold on;
%         for i = 2:num_agents
%             % color = colors{mod(i-1, length(colors)) + 1};
%             plot(trajectory{i}(:, 1), trajectory{i}(:, 2), 'k', 'LineWidth', 0.01,'LineStyle',"-",'HandleVisibility', 'off');
%         end
%         hold off;
% 
% end
%   
         pause(0.05)
          drawnow

          % frame = getframe(gcf); 
          % writeVideo(v, frame); 


       
        A = norm(leader_pos - end_pos) >= distance_threshold;
        B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) >= form_goal);

        % set limited time
        steps = steps + 1;
        if steps >= 700
            break
        end

    end
 

actual_time = (0:actual_time_step:(steps-1)*actual_time_step);
figure

for i = 1:num_agents-1
   
    current_steps = min(steps, size(error_history, 2));
    plot(actual_time(1:current_steps), error_history(i, 1:current_steps), 'LineWidth', 1.5);
    hold on;
end
xlim([0, 30])
title('Normalized Error Norm of Each Agent Over Time',"FontSize",17);
xlabel('Time (s)',"FontSize",17); 
ylabel('||  formation tracking error ||',"FontSize",17);
legend('F1', 'F2', 'F3', 'F4','F5',"F6",'FontSize', 17, 'Box', 'on', 'Location', 'best');
hold off;
set(gcf, 'Position',  [400, 400, 600, 600])

    % close(v);
clear;
close all;
clc;
% v = VideoWriter('APF6.mp4', 'MPEG-4');
% v.FrameRate = 24; 
% open(v);

num_agents = 7;
num_obstacles = 5;
[x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_agents);

[x_obs, y_obs, r_obs] = obstacles_initial (num_obstacles);


K_controller_formation = 1;
d_max_repulsion_obs = 70;    
d_max_repulsion_agent = 9; 
k_repulsion_obs=40000;               
k_repulsion_agent = 0;
safe_dis_agent = 2;
e_form = 0.05;
repulsion_max_obs = 5; 
repulsion_max_agent = 1;
zeta_agent = 0.0005;
side_length =30;



[target_position, delta_agent_x , delta_agent_y] = formation_hex_shape(x_leader,y_leader,side_length);

delta_xy= [delta_agent_x, delta_agent_y];

target_position_tem = target_position;
% Adjacent matrix  a(ij)
adj_matrix=[0 1 1 0 0 ;                 
            0 0 0 0 2 ;
            0 0 0 2 0 ;
            0 0 0 0 1 ;
            0 0 0 0 0 ;];


follower_pos =[x' , y'];
leader_pos  =[x_leader, y_leader];

all_agent = [leader_pos;follower_pos];

obst_pos    = [x_obs', y_obs'];
end_pos    =[200,400];




[x1,y1] = draw_circle_f(follower_pos(:,1),follower_pos(:,2),r );
 [xl,yl] = draw_circle_f(leader_pos(:,1),leader_pos(:,2),r_leader);


 [xob, yob, fill_handles] = draw_circle_obs(obst_pos(:,1), obst_pos(:,2), r_obs);

 
[x_end,y_end] = draw_circle_f(end_pos(1),end_pos(2),3);        

[x_tar,y_tar] = draw_circle_f(target_position(:,1),target_position(:,2),r);


map.agentx = x1;
map.agenty = y1;
map.leaderx = xl;
map.leadery = yl;
map.obstaclex = xob;
map.obstacley = yob;
map.obstacleFills = fill_handles;
map.endx = x_end;
map.endy = y_end; 
%
map.tarx = x_tar;
map.tary = y_tar;



 markers = {'o'};
 
 colors = {[255/255, 0/255, 0/255], [35/255, 85/255, 206/255], [114/255, 47/255, 16/255], ...
     [30/255, 76/255, 99/255],[255/255, 165/255, 0/255],[114/255, 156/255, 68/255]};

     colorss = {[255/255, 0/255, 0/255], [160/255, 160/255, 160/255], [114/255, 47/255, 16/255], ...
      [30/255, 76/255, 99/255],[255/255, 165/255, 0/255],[114/255, 156/255, 68/255]};
    


    for j = 1:1
        hold on
        plot_leader = plot(map.leaderx{j},map.leadery{j},'r',LineWidth=2);
        robot_struct.rleader(1) = plot_leader;
    end






     for j = 1:num_obstacles
        hold on
        plot_o = plot(map.obstaclex{j},map.obstacley{j},'K',LineWidth=2);
        obstacle_struct.o(j) = plot_o;
        hold off
    end

  
for j = 1:num_obstacles
    obstacle_struct.o1(j) = fill_handles(j);
end

 
title(' Obstacle Avoidance via APF method', 'FontSize', 17);  
xlabel('X position ', 'FontSize', 17);
ylabel('Y position ', 'FontSize', 17);


legend_labels = {'Agent', 'Obs'};


legend_handles = gobjects(2 , 1); % +1 for the Obstacle

hold on; 
for i = 1:2
  
    legend_handles(i) = plot(NaN, NaN, 'MarkerEdgeColor', 'k', 'Color', colorss{i}, 'MarkerSize', 10,LineWidth=2);
end

hLegend = legend(legend_handles, legend_labels);


set(hLegend, 'FontSize', 12, 'Box', 'on');
hold off; 


agent_trajectories(num_agents) = struct('x', [], 'y', []);






xlim([-200, 300]);
ylim([250, 350]);  


set(gcf, 'Position',  [400, 400, 500, 500])  


axis equal;

grid on;
box on;


    %% Potential Field + formation control Planner

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



    trajectory = cell(num_agents, 1); 
for i = 1:num_agents
    trajectory{i} = [all_agent(i, :)];
end








    %%
    while A+B ~=0  

 


        [target_position,~, ~] = formation_hex_shape(leader_pos(1),leader_pos(2),side_length);

        % Parameters for the sine wave

    F_attr_leader = AttractiveF(leader_pos,end_pos,0.3,3);
    repulsion_force_leader = [0, 0];

    for k = 1:num_obstacles
        repulsion_force_leader = repulsion_force_leader + ...
            compute_repulsion_new(leader_pos, obst_pos(k,:), ...
            repulsion_max_obs, safe_dis_agent, k_repulsion_obs, ...
            d_max_repulsion_obs, r_leader, r_obs(k));
    end


    % for k = 1:num_agents
    %    repulsion_force_leader_agent = compute_repulsion_new(leader_pos,follower_pos(k,:),repulsion_max_agent,safe_dis_agent,k_repulsion_agent, d_max_repulsion_agent,r(i),r(m));
    %                % function [repulsion] = compute_repulsion_new(robot_pose, obs_pose, repulsion_max, d_safe, k_rep, d_max,radius_agent,obs_radius)
    %
    % end

    velocity_leader = F_attr_leader + repulsion_force_leader; %+ repulsion_force_leader_agent;

    leader_pos = leader_pos + velocity_leader;
    disp(velocity_leader)



       




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
               
           
                position_error = (follower_pos(i,:) - target_position(i,:));
                %position_error = target_position - coord(i, :); % 
                F_formation = (- zeta_agent) * position_error;

                



                %%
                F_totx = (F_formation(1))+F_rep_agent_sum(1)+F_rep_obs_sum(1); %Add all forces in x
                
                F_toty = (F_formation(2))+F_rep_agent_sum(2)+F_rep_obs_sum(2); %Add all forces in y
               
 
           end

        current_error = norm(follower_pos(i,:) - target_position(i,:));
        error_history(i, steps) = current_error;
        
  
       end

      
         robot_struct.rleader(1).XData = robot_struct.rleader(1).XData+velocity_leader(1);
         robot_struct.rleader(1).YData = robot_struct.rleader(1).YData+velocity_leader(2);


    %%

    
        trajectory{1} = [trajectory{1}; leader_pos];
 


%%
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

        hold on;
        for i = 1
            color = colors{mod(i-1, length(colors)) + 1};
           plot(trajectory{i}(:, 1), trajectory{i}(:, 2), 'Color', color, 'LineWidth', 1.5,'LineStyle',":",'HandleVisibility', 'off');
        end
        hold off;



%   
         pause(0.05)
          drawnow

          % frame = getframe(gcf); 
          % writeVideo(v, frame); 

        A = norm(leader_pos - end_pos) >= distance_threshold;
        B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) >= form_goal);

        steps = steps + 1;
        if steps >= 300
            break
        end

    end


     % close(v);
clear;
close all;
clc;


num_agents = 7;
num_obstacles = 5;
[x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_agents);

[x_obs, y_obs, r_obs] = obstcales_initial1 (num_obstacles);


K_controller_formation = 1;
d_max_repulsion_obs = 9;    
d_max_repulsion_agent = 25; 
k_repulsion_obs=0;               
k_repulsion_agent = 16000;
safe_dis_agent = 2;
e_form = 0.05;
repulsion_max_obs = 1; 
repulsion_max_agent = 4;
zeta_agent = 0.5;%
side_length =60;


[target_position, delta_agent_x , delta_agent_y] = formation_hex_shape(x_leader,y_leader,side_length);

delta_xy= [delta_agent_x, delta_agent_y];

target_position_tem = target_position;

follower_pos =[x' , y'];
leader_pos  =[x_leader, y_leader];

all_agent = [leader_pos;follower_pos];

obst_pos    = [x_obs', y_obs'];
end_pos    =[600,200];




[x1,y1] = draw_circle_f(follower_pos(:,1),follower_pos(:,2),r );
 [xl,yl] = draw_circle_f(leader_pos(:,1),leader_pos(:,2),r_leader);

 [xob,yob] = draw_rectangle_f(obst_pos(:,1),obst_pos(:,2),r_obs,r_obs,4);
 
[x_end,y_end] = draw_circle_f(end_pos(1),end_pos(2),3);  

[x_tar,y_tar] = draw_circle_f(target_position(:,1),target_position(:,2),r);


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
 
 colors = {[89/255, 30/255, 120/255], [35/255, 85/255, 206/255], [114/255, 47/255, 16/255], ...
     [30/255, 76/255, 99/255],[201/255, 159/255, 53/255],[114/255, 156/255, 68/255]};
%
 
    for j = 1:num_agents - 1
        hold on
     
        plot_follower = plot(map.agentx{j},map.agenty{j},'b',LineWidth=2);
        robot_struct.rfollower(j) = plot_follower;
    end

    legend_handle_leader = plot_follower(1); 

    colorss = {[89/255, 30/255, 120/255], [35/255, 85/255, 206/255], [114/255, 47/255, 16/255], ...
     [30/255, 76/255, 99/255],[201/255, 159/255, 53/255],[114/255, 156/255, 68/255]};
 



    for j = 1:1
        hold on
        plot_leader = plot(map.leaderx{j},map.leadery{j},'r',LineWidth=2);
        robot_struct.rleader(1) = plot_leader;
    end

    for j = 1:num_obstacles
        hold on
        
        plot_o = plot(map.obstaclex{j},map.obstacley{j},'k',LineWidth=1);
        obstacle_struct.o(j) = plot_o;
        hold off
    end

     legend_handle_obstacle = plot_o(1);


agent_trajectories(num_agents) = struct('x', [], 'y', []);


snapshot_steps = [ 40, 180, 315, 450,585];  



    xlim([-250, 300]);
    ylim([-150, 400]);
    % figure;
    set(gcf, 'Position',  [400, 400, 550, 550])
    axis equal;
    grid on
    box on


   

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



    trajectory = cell(num_agents, 1); 
for i = 1:num_agents
    trajectory{i} = [all_agent(i, :)];
end



    %%
    while A+B ~=0 


        
      if steps >= 120
         
           
    zeta_agent = 0.08;
  
    failed_agents_indices = [2, 3, 4]; 


    all_agents_indices = 1:size(follower_pos, 1);

    valid_agents_indices = setdiff(all_agents_indices, failed_agents_indices);

  
    for i = failed_agents_indices
        target_position(i, :) = follower_pos(i, :);
    end

   
    if ~isempty(valid_agents_indices)
    
        [new_target_positions, ~, ~] = formation_triangle_shape(leader_pos(1), leader_pos(2), side_length);
       
        for i = 1:length(valid_agents_indices)
            target_position(valid_agents_indices(i), :) = new_target_positions(i, :);
        end
    end
      else
   
    [target_position, ~, ~] = formation_hex_shape(leader_pos(1), leader_pos(2), side_length);
end




        
       
            F_attr_leader = AttractiveF(leader_pos,end_pos,0.008,1);
            repulsion_force_leader = [0, 0];
          
            for k = 1:num_obstacles
                repulsion_force_leader = repulsion_force_leader + ...
                                         compute_repulsion_new(leader_pos, obst_pos(k,:), ...
                                         repulsion_max_obs, safe_dis_agent, k_repulsion_obs, ...
                                         d_max_repulsion_obs, r_leader, r_obs(k));
            end
    
           
    
    
    
            velocity_leader = F_attr_leader + repulsion_force_leader; %+ repulsion_force_leader_agent;
    
            leader_pos = leader_pos + velocity_leader;
            disp(velocity_leader)
       failed_agents_indices = [1, 5, 6]; 

       for i = 1:num_agents-1
           
           if sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) > form_goal) ~= 0
        
                a= abs(vecnorm(follower_pos(i)')-vecnorm(target_position(i)'));
                %disp(a)
                %For all obstacles in the map, calculate repulsive force
                %between robot and obstacle
                for k = 1:num_obstacles
                     F_rep_obs(k,:) = compute_repulsion_new(follower_pos(i,:),obst_pos(k,:),repulsion_max_obs,safe_dis_agent, k_repulsion_obs, d_max_repulsion_obs,r(i),r_obs(k));                                           
                end
                
                %For all other robots m in the map, calculate repulsive
                %force between robot m and robot i
                for m = 1:6
                    if m == i || any(m == failed_agents_indices)
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
              
                F_formation = (- zeta_agent) * position_error - 0.002*sign(position_error);



          
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
        

       end

     
         robot_struct.rleader(1).XData = robot_struct.rleader(1).XData+velocity_leader(1);
         robot_struct.rleader(1).YData = robot_struct.rleader(1).YData+velocity_leader(2);


 

        trajectory{1} = [trajectory{1}; leader_pos];
        
        for i = 2:num_agents
            trajectory{i} = [trajectory{i}; follower_pos(i-1, :)];
        end


    [obst_pos,obstacle_struct] = move_obstacle(obst_pos,5,obstacle_struct,r_obs(5));


 
 all_agent = [leader_pos; follower_pos];


   if steps < 123
   

  line_index = 1; 
for g = 1:length(groups)
    group = groups{g};
    for i = 1:length(group) - 1
        current_i = group(i);
        next_i = group(i + 1);
        


        if isgraphics(agent_lines(line_index)) 
            if steps == 122
                set(agent_lines(line_index), 'XData', [all_agent(current_i,1), all_agent(next_i,1)], ...
                                         'YData', [all_agent(current_i,2), all_agent(next_i,2)],'Visible', 'off');

            end
            set(agent_lines(line_index), 'XData', [all_agent(current_i,1), all_agent(next_i,1)], ...
                                         'YData', [all_agent(current_i,2), all_agent(next_i,2)]);
        end
        line_index = line_index + 1; 
    end

end
    end

if steps >= 125
    groups = {[2, 7, 6, 2]};
 line_index = 1; 
for g = 1:length(groups)
    group = groups{g};
    for i = 1:length(group) - 1
        current_i = group(i);
        next_i = group(i + 1);
       
        if isgraphics(agent_lines(line_index))
            set(agent_lines(line_index), 'XData', [all_agent(current_i,1), all_agent(next_i,1)], ...
                                         'YData', [all_agent(current_i,2), all_agent(next_i,2)],'Visible', 'on');
        end
        line_index = line_index + 1; 
    end
    
end
end

         pause(0.05)
          drawnow

 
        A = norm(leader_pos - end_pos) >= distance_threshold;
        B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) >= form_goal);

      
        steps = steps + 1;
        if steps >= 500
            break
        end

    end
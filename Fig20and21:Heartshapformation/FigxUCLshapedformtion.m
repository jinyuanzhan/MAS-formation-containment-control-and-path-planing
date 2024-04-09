clear all;
close all;
clc;




num_agents = 22;
num_obstacles = 17;
[x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_agents);

[x_obs, y_obs, r_obs] = obstacles_initial (num_obstacles);


K_controller_formation = 1;
d_max_repulsion_obs = 80;    
d_max_repulsion_agent = 15; 
k_repulsion_obs=40000;               
k_repulsion_agent = 1600;
safe_dis_agent = 1;
e_form = 0.05;
repulsion_max_obs = 10; 
repulsion_max_agent = 1;
zeta_agent = 0.15;
side_length =40;


[target_position, delta_agent_x , delta_agent_y] = formation_u_shape(x_leader,y_leader,side_length);

delta_xy= [delta_agent_x, delta_agent_y];

target_position_tem = target_position;

adj_matrix=[0 1 1 0 0 ;                 
            0 0 0 0 2 ;
            0 0 0 2 0 ;
            0 0 0 0 1 ;
            0 0 0 0 0 ;];

%follower_pos_tem = [x',y'];
follower_pos =[x' , y'];
leader_pos  =[x_leader, y_leader];

all_agent = [leader_pos;follower_pos];

obst_pos    = [x_obs', y_obs'];
end_pos    =[1000,100];


title('"UCL" Formation Control with Dense Obstacles ','FontSize', 20);


[x1,y1] = draw_circle_f(follower_pos(:,1),follower_pos(:,2),r );
[xl,yl] = draw_circle_f(leader_pos(:,1),leader_pos(:,2),r_leader);
[xob,yob] = draw_circle_f(obst_pos(:,1),obst_pos(:,2),r_obs);
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


    for j = 1:num_agents - 1
        hold on
      
        plot_follower = plot(map.agentx{j},map.agenty{j},'k',LineWidth=2);
        robot_struct.rfollower(j) = plot_follower;
    end
    
 



    for j = 1:num_obstacles
        hold on
        R = 72 / 255;
        G = 127 / 255;
        B = 195 / 255;
        plot_o = plot(map.obstaclex{j},map.obstacley{j},'Color', [R, G, B],LineWidth=2);
        obstacle_struct.o(j) = plot_o;
        hold off
    end

    xlim([-300, 900]);
    ylim([0, 350]);
    
    % figure;
    set(gcf, 'Position',  [400, 400, 1100, 500])
    grid on
    box("on")



    




    steps = 1;
   
    F_rep_obs = zeros(num_obstacles,2);

    F_rep_aget = zeros(num_agents,2);

    form_goal = e_form*ones(num_agents-1,1)';

    f_goal = 1; 
    
   
    A = ((abs(norm(leader_pos)-norm(end_pos)) > f_goal)); %Check if formation is at goal
    
    B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) > form_goal); %Check if all robots are at their formation goal locations
    
  

    distance_threshold = 1.0;
    
   
    error_history = zeros(num_agents-1, 2800); 

groups = {[2:8], [9:15], [16:22]};

agent_lines = gobjects(sum(cellfun(@numel, groups)) - length(groups), 1);

line_index = 1; 

hold on; 
for g = 1:length(groups)
    group = groups{g}; 
    for i = 1:length(group) - 1
        current_i = group(i);
        next_i = group(i + 1); 
     
        R = 205 / 255;
        G = 131 / 255;
        B = 135 / 255;
        
        agent_lines(line_index) = plot([all_agent(current_i,1), all_agent(next_i,1)], ...
                                       [all_agent(current_i,2), all_agent(next_i,2)], 'k-', 'LineWidth', 4, 'Color', [R, G, B]);
        
        line_index = line_index + 1; 
    end
end
hold off;


    %%

    trajectory = cell(num_agents, 1); 
for i = 1:num_agents
    trajectory{i} = [all_agent(i, :)];
end



    %%
    while A+B ~=0  

        [target_position,~, ~] = formation_u_shape(leader_pos(1),leader_pos(2),side_length);
        
       
            F_attr_leader = AttractiveF(leader_pos,end_pos,0.02,1);
            repulsion_force_leader = [0, 0];
           
            for k = 1:num_obstacles
                repulsion_force_leader = repulsion_force_leader + ...
                                         compute_repulsion_new(leader_pos, obst_pos(k,:), ...
                                         repulsion_max_obs, safe_dis_agent, k_repulsion_obs, ...
                                         2.5, r_leader, r_obs(k));
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
                
                for k = 1:num_obstacles
                     F_rep_obs(k,:) = compute_repulsion_new(follower_pos(i,:),obst_pos(k,:),6,safe_dis_agent, k_repulsion_obs, d_max_repulsion_obs,r(i),r_obs(k));                                           
                end
                
               
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
              


                position_error = (follower_pos(i,:) - target_position(i,:));
                %position_error = target_position - coord(i, :);
                F_formation = (- zeta_agent) * position_error;



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
        
   
       end

       
        trajectory{1} = [trajectory{1}; leader_pos];
        
       
        for i = 2:num_agents
            trajectory{i} = [trajectory{i}; follower_pos(i-1, :)];
        end

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



         pause(0.05)
          drawnow
        

        
       
        A = norm(leader_pos - end_pos) >= distance_threshold;
        B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) >= form_goal);

       
        steps = steps + 1;
        if steps >= 700
            break
        end

    end
    end

%close(v);


clear all;
close all;
clc;


% v = VideoWriter('Heartformation.mp4', 'MPEG-4');
% v.FrameRate = 24; 
% open(v);

%v = VideoWriter('myAnimation.mp4', 'MPEG-4');


num_agents = 11;
num_obstacles = 60;
[x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_agents);



[x_obs, y_obs, r_obs] = obstacles_initial (num_obstacles);


K_controller_formation = 1;
d_max_repulsion_obs = 40;    
d_max_repulsion_agent = 15; 
k_repulsion_obs=20000;               
k_repulsion_agent = 1600;
safe_dis_agent = 1;
e_form = 0.05;
repulsion_max_obs = 7; 
repulsion_max_agent = 1;
zeta_agent = 0.15;
side_length =40;


[target_position, delta_agent_x , delta_agent_y] = formation_heart_shape(x_leader,y_leader,side_length);

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
end_pos    =[1000,200];

title('"Heart-Shaped" Formation Control in Unknown Dense Environment ','FontSize', 17);



%% Plot robots, obstacles, goal

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
       % plot_end = plot(map.endx{1},map.endy{1},'g',LineWidth=2);
        plot_follower = plot(map.agentx{j},map.agenty{j},'k',LineWidth=2);
        robot_struct.rfollower(j) = plot_follower;
    end
    
 legend_handle_leader = plot_follower(1);  



    for j = 1:num_obstacles
        hold on
        R = 72 / 255;
        G = 127 / 255;
        B = 195 / 255;
        plot_o = plot(map.obstaclex{j},map.obstacley{j},'Color', [R, G, B],LineWidth=2);
        obstacle_struct.o(j) = plot_o;
        hold off
    end

    legend_handle_obstacle = plot_o(1);  

    


    xlim([-600, 1200]);
    ylim([0, 350]);
    
   
    set(gcf, 'Position',  [400, 300, 1300, 420])
 
    box("on")

   
legend([legend_handle_leader, legend_handle_obstacle], {'Robot', 'Obstacle'});


set(legend, 'FontSize', 15, 'Box', 'on');


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

groups = {[2:11, 2]};  

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
                                       [all_agent(current_i,2), all_agent(next_i,2)], 'k-', 'LineWidth', 4, 'Color', [R, G, B], 'HandleVisibility', 'off');

        line_index = line_index + 1; 
    end
end
hold off;


    %%



colors = lines(num_agents);  


trajectory = cell(num_agents, 1); 
for i = 1:num_agents
    trajectory{i} = [all_agent(i, :)];
end

    speed_leader = 1;
    
    direction = (end_pos - leader_pos) / norm(end_pos - leader_pos);
 
    velocity_leader = speed_leader * direction;





    while A+B ~=0 
        %legend('Robots',  'Obstacle');


        [target_position,~, ~] = formation_heart_shape(leader_pos(1),leader_pos(2),side_length);
        

    
            leader_pos = leader_pos + velocity_leader;
            disp(velocity_leader)

            
       

       for i = 1:num_agents-1
          
           if sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) > form_goal) ~= 0
           %if abs(vecnorm(follower_pos')-vecnorm(target_position')) > form_goal ~= 0

                a= abs(vecnorm(follower_pos(i)')-vecnorm(target_position(i)'));
                
                for k = 1:num_obstacles
                     F_rep_obs(k,:) = compute_repulsion_new(follower_pos(i,:),obst_pos(k,:),4,safe_dis_agent, k_repulsion_obs, d_max_repulsion_obs,r(i),r_obs(k));                                           
                end
                
               
                for m = 1:num_agents-1
                    if m == i 
                        continue
                    else
                        F_rep_aget(m,:) = compute_repulsion_new(follower_pos(i,:),follower_pos(m,:),repulsion_max_agent,safe_dis_agent,k_repulsion_agent, d_max_repulsion_agent,r(i),r(m));                                               
                     
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
   
    % 
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
end

drawnow; 


if steps > 700
    hold on;
    for i = 2:num_agents
        
        plot(trajectory{i}(:, 1), trajectory{i}(:, 2), 'LineWidth', 0.3, 'Color', colors(i, :), 'LineStyle', '--', 'HandleVisibility', 'off');
    end
    hold off;
end


         pause(0.05)
          drawnow
          % frame = getframe(gcf); 
          % writeVideo(v, frame); 

        
     
        A = norm(leader_pos - end_pos) >= distance_threshold;
        B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) >= form_goal);

  
        steps = steps + 1;
        if steps >= 1400
            break
        end

    end
 


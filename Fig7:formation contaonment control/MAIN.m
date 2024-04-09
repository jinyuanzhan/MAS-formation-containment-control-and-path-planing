clear;
close all;
clc;
% v = VideoWriter('Circile_trajectory.mp4', 'MPEG-4');
% v.FrameRate = 24; %
% open(v);

num_agents = 5;
num_obstacles = 5;
num_lost=2;
num_follo = 4;
[x_lost,y_lost,r_lost] = lost_robo_initial(num_lost);



[x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_agents);

[x_obs, y_obs, r_obs] = obstacles_initial (num_obstacles);
p1=500;

K_controller_formation = 1;
d_max_repulsion_obs = 9;    
d_max_repulsion_agent = 9; 
k_repulsion_obs=0;               
k_repulsion_agent = 0;
safe_dis_agent = 2;
e_form = 0.05;
repulsion_max_obs = 1; 
repulsion_max_agent = 1;
zeta_agent = 0.07;
side_length =90;
 steps = 1;
 actual_time_step = 0.06; 
yita=0.002;
[target_position, delta_agent_x , delta_agent_y] = formation_hex_shape(x_leader,y_leader,side_length);

delta_xy= [delta_agent_x, delta_agent_y];

containment_sclar_lost = 0.1;


target_position_tem = target_position;

adj_matrix=[0 1 1 0 0 ;                 
            0 0 0 0 2 ;
            0 0 0 2 0 ;
            0 0 0 0 1 ;
            0 0 0 0 0 ;];


follower_pos =[x' , y'];
leader_pos  =[x_leader, y_leader];
lost_pos = [x_lost' , y_lost'];

all_agent = [leader_pos;follower_pos;lost_pos];

obst_pos    = [x_obs', y_obs'];
end_pos    =[600,200];



[x1,y1] = draw_circle_f(follower_pos(:,1),follower_pos(:,2),r );
 [xl,yl] = draw_circle_f(leader_pos(:,1),leader_pos(:,2),r_leader);
 
 [xob,yob] = draw_circle_f(obst_pos(:,1),obst_pos(:,2),r_obs);

 [xlost,ylost] = draw_circle_f(lost_pos(:,1),lost_pos(:,2),r_lost);
 
[x_end,y_end] = draw_circle_f(end_pos(1),end_pos(2),1);     

[x_tar,y_tar] = draw_circle_f(target_position(:,1),target_position(:,2),r);


map.agentx = x1;
map.agenty = y1;
map.leaderx = xl;
map.leadery = yl;
map.lostx = xlost;
map.losty = ylost;
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


 
    for j = 1:num_agents - 1
        hold on
       % plot_end = plot(map.endx{1},map.endy{1},'g',LineWidth=2);
        plot_follower = plot(map.agentx{j},map.agenty{j},'b',LineWidth=2);
        robot_struct.rfollower(j) = plot_follower;
    end

    legend_handle_leader = plot_follower(1); 

    colorss = {[255/255, 0/255, 0/255], [35/255, 85/255, 206/255], [114/255, 47/255, 16/255], ...
     [30/255, 76/255, 99/255],[255/255, 165/255, 0/255],[114/255, 156/255, 68/255]};

   

    for j = 1:1
        hold on
        plot_leader = plot(map.leaderx{j},map.leadery{j},'r',LineWidth=2);
        robot_struct.rleader(1) = plot_leader;
    end



for j = 1:num_lost
    hold on
    plot_lostrobo = plot(map.lostx{j}, map.losty{j}, 'Color', [0.6500, 0.3250, 0.0980], 'LineWidth', 2);
    robot_struct.rlostrobo(j) = plot_lostrobo;
end



    for j = 1:num_obstacles
        hold on

        plot_o = plot(map.obstaclex{j},map.obstacley{j},'r',LineWidth=0.5);
        obstacle_struct.o(j) = plot_o;
        hold off
    end

    

time_step = 0.1; 
time = (0:time_step:(steps-1)*time_step);


title('Formation-Containment Control', 'FontSize', 23);  
xlabel('X position ', 'FontSize', 20);
ylabel('Y position ', 'FontSize', 20);

set(gca, 'LineWidth', 0.7);
legend_labels = {'Leader', 'Follower',"Virtual target"};


legend_handles = gobjects(3 , 1); % +1 for the Obstacle
colorsses = {[0/255, 0/255, 255/255],[0.6500, 0.3250, 0.0980],[1, 0, 0]};

hold on; 
for i = 1:3
   
    legend_handles(i) = plot(NaN, NaN, 'MarkerEdgeColor', 'k', 'Color', colorsses{i}, 'MarkerSize', 10,LineWidth=2);
end

hLegend = legend(legend_handles, legend_labels);


set(hLegend, 'FontSize', 18, 'Box', 'on');
hold off; 

agent_trajectories(num_agents) = struct('x', [], 'y', []);


 snapshot_steps = [ 60, 220, p1]; 

new_xlim = [-300, 700];


original_width = 400 - (-300);  
original_height = 500 - (-100);  


new_width = new_xlim(2) - new_xlim(1); 

new_height = (new_width * original_height) / original_width; 


ylim_center = (-100 + 500) / 2;
new_ylim = [ylim_center - new_height/2, ylim_center + new_height/2];


xlim(new_xlim);
ylim(new_ylim);


set(gcf, 'Position',  [400, 400, 900, (900 * original_height) / original_width]);



grid on;
box on;


   

    steps = 1;
   
    F_rep_obs = zeros(num_obstacles,2);

    F_rep_aget = zeros(num_agents,2);

    form_goal = e_form*ones(num_agents-1,1)';

    f_goal = 1; 


   
    A = ((abs(norm(leader_pos)-norm(end_pos)) > f_goal)); %Check if formation is at goal
    
    B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) > form_goal); %Check if all robots are at their formation goal locations
    
    

    distance_threshold = 1.0;
    
  
    error_history = zeros(num_lost, 2800); 


  
groups = {[2:5,2]};
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

 trajectory = cell(num_agents, 1); 
for i = 1:7
    trajectory{i} = [all_agent(i, :)];
end

    lost_trajectories = cell(num_lost, 1);
for i = 1:num_lost
    lost_trajectories{i} = [lost_pos(i,:)]; 
end
    

delta_theta = 2 * pi / 470; 

x_center = -10;
y_center = 200;
radius = sqrt((x_leader - x_center)^2 + (y_leader - y_center)^2);
theta = atan2(y_leader - y_center, x_leader - x_center);

amplitude = 100; 
frequency = 0.07; 
speed_x = 26; 
initial_x = leader_pos(1); 
initial_y = leader_pos(2); 



normalized_error_norm = zeros(num_agents-1, 2800);


%%  
    while A+B ~=0  
disp(steps)


        [target_position,~, ~] = formation_hex_shape(leader_pos(1),leader_pos(2),side_length);

       
    

current_time = steps * actual_time_step;


leader_pos(1) = initial_x + speed_x * current_time;


leader_pos(2) = initial_y + amplitude * sin(2 * pi * frequency * current_time);

robot_struct.rleader(1).XData = leader_pos(1);
robot_struct.rleader(1).YData = leader_pos(2);




  
       

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
                %% formation control strategy
                

                position_error = (follower_pos(i,:) - target_position(i,:));
                %position_error = target_position - coord(i, :); 
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

  
       
       end
       
    %%

        trajectory{1} = [trajectory{1}; leader_pos];
        
        for i = 2:num_agents
            trajectory{i} = [trajectory{i}; follower_pos(i-1, :)];
        end



 if steps >20      
for i = 1:num_lost
  
    e_contaiment = zeros(1, 2);
    F_rep_obs_lost = zeros(num_obstacles, 2);
    F_rep_aget_lost = zeros(num_agents-1, 2);

    F_rep_lost_lost = zeros(num_lost-1, 2);

   
    distances_to_leaders = sqrt(sum((leader_pos - lost_pos(i,:)).^2, 2));

 
    [min_distance, nearest_leader_index] = min(distances_to_leaders);

  
    if min_distance <= 300

  
distances = arrayfun(@(j) norm(lost_pos(i,:) - follower_pos(j,:)), 1:num_follo);
[~, sortedIndices] = sort(distances, 'ascend');
sc=3;
if i==1
    sc=4;
end

nearestFiveFollowers = sortedIndices(1:sc);


      
        for j = nearestFiveFollowers 
            e_contaiment = e_contaiment + (lost_pos(i,:) - follower_pos(j,:));
            disp("e"+e_contaiment); 
        end

     
  
        F_contaiment = (- containment_sclar_lost) * e_contaiment - yita *  sign(e_contaiment);

        for k = 1:num_obstacles
            F_rep_obs_lost(k,:) = compute_repulsion_new(lost_pos(i,:), obst_pos(k,:), repulsion_max_obs, safe_dis_agent, k_repulsion_obs, d_max_repulsion_obs, r_lost(i), r_obs(k));
        end

        
        for m = nearestFiveFollowers 
            F_rep_aget_lost(m,:) = compute_repulsion_new(lost_pos(i,:), follower_pos(m,:), repulsion_max_agent, safe_dis_agent, k_repulsion_agent, d_max_repulsion_agent, r_lost(i), r(m));
        end

        
        for l = 1:num_lost
            if i ~= l 
                F_rep_lost_lost(l,:) = compute_repulsion_new(lost_pos(i,:), lost_pos(l,:), repulsion_max_agent, safe_dis_agent, k_repulsion_agent, d_max_repulsion_agent, r_lost(i), r_lost(l));
            end
        end

        
        F_rep_obs_sum_lost = sum(F_rep_obs_lost, 1);
        F_rep_agent_sum_lost = sum(F_rep_aget_lost, 1);

   
        F_rep_lost_sum = sum(F_rep_lost_lost, 1);

     
        F_lost_totalx = F_contaiment(1) + F_rep_agent_sum_lost(1) + F_rep_obs_sum_lost(1) +  F_rep_lost_sum(1);
        F_lost_totaly = F_contaiment(2) + F_rep_agent_sum_lost(2) + F_rep_obs_sum_lost(2) +  F_rep_lost_sum(2);


        robot_struct.rlostrobo(i).XData = robot_struct.rlostrobo(i).XData + F_lost_totalx;
        robot_struct.rlostrobo(i).YData = robot_struct.rlostrobo(i).YData + F_lost_totaly;

        lost_pos(i,1) = lost_pos(i,1) + F_lost_totalx;
        lost_pos(i,2) = lost_pos(i,2) + F_lost_totaly;

      
lost_trajectories{i}  = [lost_trajectories{i}; lost_pos(i,:)];

current_lost_error = norm(lost_pos(i,:) - leader_pos);
    error_history(i, steps) = current_lost_error;
    
    
    if steps == 1 
        normalized_error_norm(i, steps) = 1; 
    else
       normalized_error_norm(i, steps) = norm(error_history(i, steps)) / norm(error_history(i, 1));
    end



    end

end


 end


    
hold on; 
if ismember(steps, snapshot_steps)
   
    for i = 1:num_agents
       
        agent_trajectories(i).x(end+1) = all_agent(i, 1);
        agent_trajectories(i).y(end+1) = all_agent(i, 2);

      
        marker = markers{mod(i-1, length(markers)) + 1};
        color = colors{mod(i-1, length(colors)) + 1};

  
    r_ne = 5; 

    
    rectangle('Position', [all_agent(i, 1)-r_ne, all_agent(i, 2)-r_ne, 2*r_ne, 2*r_ne], ...
             'Curvature', [1 1], 'EdgeColor', color, 'LineWidth', 2);
    
    end
     for i = 1:num_lost
      
        r_lost_ne = r_lost(i); 

      
        rectangle('Position', [lost_pos(i, 1)-r_lost_ne, lost_pos(i, 2)-r_lost_ne, 2*r_lost_ne, 2*r_lost_ne], ...
                  'Curvature', [1, 1], 'EdgeColor', [0.6500, 0.3250, 0.0980], 'LineWidth', 2);
                  
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
                [all_agent(current_i,2), all_agent(next_i,2)], 'LineStyle', '-', 'Color', "k", 'LineWidth', 1,'HandleVisibility', 'off');
        end
    end
    end
end

hold off; 


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
%
 if steps > p1-2
        
        hold on;
        for i = 1
        
            plot(trajectory{i}(:, 1), trajectory{i}(:, 2), 'Color', "r", 'LineWidth', 1,'LineStyle',"-",'HandleVisibility', 'off');
        end
        for i = 2:num_agents
           
            plot(trajectory{i}(:, 1), trajectory{i}(:, 2), 'Color', "k", 'LineWidth', 0.8,'LineStyle',"-.",'HandleVisibility', 'off');
        end
        
 end



hold on; 
for i = 1:num_lost
    if ~isempty(lost_trajectories{i}) 
        plot(lost_trajectories{i}(:, 1), lost_trajectories{i}(:, 2), 'Color', [0.6500, 0.3250, 0.0980], 'LineWidth', 1,'LineStyle',"--",'HandleVisibility', 'off');
    end
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
        if steps >= p1
            break
        end

    end
 

actual_time = (0:actual_time_step:(steps-1)*actual_time_step);
    % close(v);
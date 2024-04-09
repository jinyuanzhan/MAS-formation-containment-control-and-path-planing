clear;
close all;
clc;

  %  v = VideoWriter('', 'MPEG-4');
  %  v.FrameRate = 24;
  % open(v);


num_agents = 12; % 1 leader / 5 follower
num_obstacles = 5;
num_lost=3;
num_leader = 2;
num_follo = 10;

 actual_time_step = 0.06;

[x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_agents);

[x_lost,y_lost,r_lost] = lost_robo_initial(num_lost);

[x_obs, y_obs, r_obs] = obstacles_initial (num_obstacles);

K_controller_formation = 1;
d_max_repulsion_obs = 80;
d_max_repulsion_agent = 10;
k_repulsion_obs=40000;
k_repulsion_agent = 160;
safe_dis_agent = 1;
e_form = 0.05;
repulsion_max_obs = 5; 
repulsion_max_agent = 1;
zeta_agent = 0.3;
side_length =40;
containment_sclar_lost = 0.03;
yita= 0.002;
return_positions = [-100, 265; 
                    -100, 70]; 

%----
mu = 1; %
Q=0.00002;
R=0.0001;
ci=2;
%---
f_xi = @(xi) (norm(xi) ~= 0) * (xi / norm(xi));




[target_position, delta_agent_x , delta_agent_y] = formation_relative_to_leader_pentagon_two_lost(x_leader,y_leader,side_length);

target_postion_leader_1 = target_position(1:5, :); 
target_postion_leader_2 = target_position(6:10, :);
delta_x_leader_1 =  delta_agent_x(1:5);
delta_y_leader_1 = delta_agent_y(1:5);
delta_x_leader_2 = delta_agent_x(6:10);
delta_y_leader_2 = delta_agent_y(6:10);


target_position_tem = target_position;

%not used we set resecuer fully communicate with virtual target or each
%orther.


adj_matrix=[0 1 1 0 0 1;
    0 0 0 0 2 0;
    0 0 0 2 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];
%          leader  agent1  agent2  agent3  agent4
% leader      0       1       1       0       0
% agent1      0       0       0       0       1
% agent2      0       0       0       1       0
% agent3      0       0       0       0       0
% agent4      0       0       0       0       0




follower_pos =[x' , y'];
leader_pos  =[x_leader, y_leader];

lost_pos = [x_lost' , y_lost'];

all_agent = [leader_pos;follower_pos; lost_pos];

obst_pos    = [x_obs', y_obs'];

end_pos    = [(lost_pos(1,1)-20),(lost_pos(1,2)+20);(lost_pos(2,1)-20),(lost_pos(2,2)+20)];

[x1,y1] = draw_circle_f(follower_pos(:,1),follower_pos(:,2),r );
[xl,yl] = draw_circle_f(leader_pos(:,1),leader_pos(:,2),r_leader);
[xlost,ylost] = draw_circle_f(lost_pos(:,1),lost_pos(:,2),r_lost);

[x_end,y_end] = draw_circle_f(end_pos(:,1),end_pos(:,2),[3,3]);         

[x_tar,y_tar] = draw_circle_f(target_position(:,1),target_position(:,2),r);


map.agentx = x1;
map.agenty = y1;
map.leaderx = xl;
map.leadery = yl;
map.lostx = xlost;
map.losty = ylost;
% map.obstaclex = xob;
% map.obstacley = yob;
map.endx = x_end;
map.endy = y_end;
%
map.tarx = x_tar;
map.tary = y_tar;

map.obstacles = repmat(struct('X', [], 'Y', [], 'Z', []), num_obstacles, 1);


for j = 1:num_follo
    hold on
   % plot_end = plot(map.endx{1},map.endy{1},'g',LineWidth=2);
   % plot_end = plot(map.endx{2},map.endy{2},'g',LineWidth=2);

    %plot_follower = plot(map.agentx{j},map.agenty{j},'b',LineWidth=2);
    % robot_struct.rfollower(j) = plot_follower;

    plot_follower = plot(follower_pos(j,1), follower_pos(j,2), 'Marker', 'o',  'MarkerSize', 8, 'LineWidth', 1);
    robot_struct.rfollower(j) = plot_follower;
end



for j = 1:num_lost
    hold on
    plot_lostrobo = plot(lost_pos(j,1), lost_pos(j,2), 'Marker', '*',  'MarkerSize', 8, 'LineWidth', 1);
   % plot_lostrobo = plot(map.lostx{j}, map.losty{j}, 'Color', [0.6500, 0.3250, 0.0980], 'LineWidth', 2);
    robot_struct.rlostrobo(j) = plot_lostrobo;
end

%z_value=0.00003;

leader_markers = gobjects(num_leader, 1); 
drone_lines = gobjects(num_leader, 5); 
for j = 1:num_leader
    hold on;

    plot_leader = plot(leader_pos(j,1), leader_pos(j,2), 'r--^', 'MarkerSize', 8, 'LineWidth', 1);

   robot_struct.rleader(j) = plot_leader;
    

    leader_drones(j) = plot3(leader_pos(j,1), leader_pos(j,2), 5, 'k^', 'MarkerSize', 8, 'LineWidth', 1);
    
    for k = 1:5 
        followerIndex = (j - 1) * 5 + k; 

        drone_lines(j, k) = line([leader_pos(j, 1), follower_pos(followerIndex, 1)], ...
                                 [leader_pos(j, 2), follower_pos(followerIndex, 2)], ...
                                 [0.01, 0], 'Color', 'k', 'LineStyle', ':'); 
    end
end


if numel(r_obs) == 1
    r_obs = repmat(r_obs, num_obstacles, 1);
end


z_value = 0; 

obstacle_struct.o = gobjects(num_obstacles, 1); 

for j = 1:num_obstacles
    hold on;
    [X, Y, Z] = draw_circle_3d(obst_pos(j,1), obst_pos(j,2), z_value, r_obs(j));
    disp(max(Z, [], 'all')); 
    h = surf(X, Y, Z, 'EdgeColor', [0.5, 0.5, 0.5], 'FaceColor', 'none', 'FaceAlpha', 0.1);
 
    obstacle_struct.o(j) = h;
end

xlim([-300, 500]);
ylim([0, 375]);
zlim([0 ,15]);
% figure;
%set(gcf, 'Position',  [400, 400, 919, 400])
set(gcf, 'Position',  [400, 400, 1600, 700])
set(0, 'defaultfigurecolor', 'w');
box on
grid on

view(3);     
view(12,54);



steps = 1;

F_rep_obs = zeros(num_obstacles,2);

F_rep_aget = zeros(num_agents,2);

form_goal = e_form*ones(num_agents-2,1)';

f_goal = 1;


A = ((abs(norm(leader_pos)-norm(end_pos)) > f_goal)); %Check if formation is at goal

B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) > form_goal); %Check if all robots are at their formation goal locations





 error_history_foll = zeros(num_follo, 3800);

 error_history_lost = zeros(num_follo, 3800);

 normalized_error_norm_foll = zeros(num_follo, 3800);


 %%
error_history_contain = zeros(num_lost, 3800);
normalized_error_norm_lost = zeros(num_lost, 3800);
%%



groups = {[3:7,3],[8:12,8  ]};
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
                                       [all_agent(current_i,2), all_agent(next_i,2)], 'k--', 'LineWidth', 0.5);

        line_index = line_index + 1; 
    end
end
hold off;






    trajectory = cell(num_agents, 1); 
for i = 1:num_agents
    trajectory{i} = [all_agent(i, :)];
end

trajectory_lost = cell(num_lost, 1); 
for k = 1:num_lost
    trajectory_lost{k} = [lost_pos(k, :)];
end



%%
while A+B ~=0  
   


    distance_threshold = 1.0;
 %gradualy increse gain 
if steps < 75
    start_value = 0.001;
    end_value = 0.35;
    max_steps = 150;
    growth_rate = 2; 
    zeta_agent = start_value + (end_value - start_value) * (steps / (max_steps - 1))^growth_rate;
end


    [target_position,~, ~] = formation_relative_to_leader_pentagon_two_lost(leader_pos(:,1),leader_pos(:,2),side_length);
   
for i = 1:num_leader
    F_attr_leader = AttractiveF(leader_pos(i,:) ,end_pos(i,:),0.1,1);
    repulsion_force_leader = zeros(1, 2);

  
    for k = 1:num_obstacles
        repulsion_force = compute_repulsion_new(leader_pos(i,:), obst_pos(k,:), repulsion_max_obs, safe_dis_agent, k_repulsion_obs, d_max_repulsion_obs, r_leader(i), r_obs(k));
        repulsion_force_leader = repulsion_force_leader + repulsion_force;
    end

    velocity_leader = F_attr_leader + repulsion_force_leader;

    if steps <175
          velocity_leader = 0;
     end

    leader_pos(i,:) = leader_pos(i,:) + velocity_leader;
 
    

     set(robot_struct.rleader(i), 'XData', leader_pos(i,1), 'YData', leader_pos(i,2));

    set(leader_drones(i), 'XData', leader_pos(i,1), 'YData', leader_pos(i,2), 'ZData', 5);


    for j = 1:5 
      
        followerIndex = (i - 1) * 5 + j;

        set(drone_lines(i, j), 'XData', [leader_pos(i, 1), follower_pos(followerIndex, 1)], ...
                                'YData', [leader_pos(i, 2), follower_pos(followerIndex, 2)], ...
                                'ZData', [5, 0]); 
    end
    

end
 
for i = 1:num_follo  
    
    leaderIndex = ceil(i / (num_follo / num_leader));  
    
  
    targetForFollower = target_position(i, :);

    position_error = follower_pos(i, :) - targetForFollower;
    

    F_rep_total = zeros(1, 2);
    for k = 1:num_obstacles
        F_rep_total = F_rep_total + compute_repulsion_new(follower_pos(i, :), obst_pos(k, :), repulsion_max_obs, safe_dis_agent, k_repulsion_obs, d_max_repulsion_obs, r(i), r_obs(k));
    end
    
 
    for j = 1:num_follo
        if i ~= j
            F_rep_total = F_rep_total + compute_repulsion_new(follower_pos(i, :), follower_pos(j, :), repulsion_max_agent, safe_dis_agent, k_repulsion_agent, d_max_repulsion_agent, r(i), r(j));
        end
    end
    
  
    F_attr = -zeta_agent * position_error  - yita * sign (position_error);  
    velocity = F_attr + F_rep_total;
    
   
    follower_pos(i, :) = follower_pos(i, :) + velocity;
   

    robot_struct.rfollower(i).XData =  robot_struct.rfollower(i).XData + velocity(1);
    robot_struct.rfollower(i).YData = robot_struct.rfollower(i).YData+velocity(2);

    
    
 norm_position_error = norm(position_error);


  error_history_foll(i,steps) = norm_position_error;

  normalized_error_norm_foll (i, steps) = norm(error_history_foll(i,steps)) / norm(error_history_foll(i,steps));

    




end


hasMetCondition = false(num_lost, 1); 

for i = 1:num_lost
    e_contaiment = zeros(1, 2);
    F_rep_obs_lost = zeros(num_obstacles, 2);
    F_rep_aget_lost = zeros(num_agents-1, 2);

   
    F_rep_lost_lost = zeros(num_lost-1, 2);


    distances_to_leaders = sqrt(sum((leader_pos - lost_pos(i,:)).^2, 2));
    [min_distance, nearest_leader_index] = min(distances_to_leaders);

     if ~hasMetCondition(i)
        error_history_lost(i, steps) =norm([0,0]) ; 
    end


    
    if min_distance <= 70
         hasMetCondition(i) = true;

distances = arrayfun(@(j) norm(lost_pos(i,:) - follower_pos(j,:)), 1:num_follo);

[~, sortedIndices] = sort(distances, 'ascend');

sc = 5;
 if i == 3
     sc = 3;
 end
nearestFiveFollowers = sortedIndices(1:sc);

        for j = nearestFiveFollowers  
            e_contaiment = e_contaiment + (lost_pos(i,:) - follower_pos(j,:)); 
        end

        F_contaiment = (- containment_sclar_lost) * e_contaiment - yita * sign (e_contaiment);

     
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

        normalized_conta_error = norm(e_contaiment);
        error_history_lost(i, steps) = normalized_conta_error;
        
        end
         end
    
   





for i = 1:num_leader
    
    distance_to_end = norm(leader_pos(i,:) - end_pos(i,:));
    
    
    if distance_to_end < 1
       
        end_pos(i,:) = return_positions(i,:); 
    end
end
  

 

all_agent = [leader_pos; follower_pos];

if steps > 5
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
end

   
if steps > 600
               
        hold on;
        for i = 2:num_agents
            if steps > 50
               % plot(trajectory{i}(:, 1), trajectory{i}(150:end, 2), 'LineWidth', 1.5,Color= "red" );
                plot(trajectory{i}(50:end, 1), trajectory{i}(50:end, 2), 'LineWidth', 0.5, 'Color', "blue","LineStyle","-");
            end
        end
        hold off;
     
        hold on;
        for k = 1:num_lost
            plot(trajectory_lost{k}(:, 1), trajectory_lost{k}(:, 2), 'LineWidth', 0.5, 'Color', "red");
        end
        hold off;

end

    
    pause(0.05)
    
    
    drawnow
       % 
       % frame = getframe(gcf); 
       %   writeVideo(v, frame); 

    A = norm(leader_pos - end_pos) >= distance_threshold;
    B = sum(abs(vecnorm(follower_pos')-vecnorm(target_position')) >= form_goal);


    steps = steps + 1;
    if steps >= 1600
        break
    end
disp(steps)

end

actual_time = (0:actual_time_step:(steps-1)*actual_time_step);

figure;
set(gcf, 'Position', [100, 100, 744, 354]); 

for i = 1:num_follo
   
    current_steps = min(steps, size(error_history_foll, 2));
   
    plot(actual_time(1:current_steps), error_history_foll(i,1:current_steps), 'LineWidth', 2);
    hold on;
end


xlabel('Time (s)', 'FontSize', 15);
ylabel('|| formation tracking error ||', 'FontSize', 15); 

legend('rescuer 1', 'rescuer 2', 'rescuer 3', 'rescuer 4', "rescuer 5", "rescuer 6", "rescuer 7",...
    "rescuer 8", "rescuer 9", "rescuer 10", 'FontSize', 15, 'Box', 'on', 'Location', 'best');

hold off;
%%
figure;
set(gcf, 'Position', [700, 100, 744, 354]); 
for i = 1:num_lost

    current_steps = min(steps, size(error_history_lost, 2));
    
    plot(actual_time(1:current_steps), error_history_lost(i,1:current_steps), 'LineWidth', 2);
    hold on;
end


xlabel('Time (s)', 'FontSize', 15); 
ylabel('|| Containment  error ||', 'FontSize', 15); 
%ylabel('||Containment error $$ \zeta_{\mathrm{lost\ robot}} $$||', 'FontSize', 18);
legend('lost robot 1', 'lost robot 2', 'lost robot 3','FontSize', 15, 'Box', 'on', 'Location', 'best');

% close(v);
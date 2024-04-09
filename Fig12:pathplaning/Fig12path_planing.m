clear;
close all;
clc;


num_agents = 8;
num_obstacles = 5;
[x,y,r,x_leader,y_leader,r_leader] = robot_initial(num_agents);

num_leader = 4;

[x_obs, y_obs, r_obs] = obstacles_initial (num_obstacles);


K_controller_formation = 1;
d_max_repulsion_obs = 250;    
d_max_repulsion_agent = 250; 
k_repulsion_obs=40000;               
k_repulsion_agent = 16000;
safe_dis_agent = 10;
e_form = 0.05;
repulsion_max_obs = 5; 
repulsion_max_agent = 5;
zeta_agent = 0.0005;
side_length =30;




adj_matrix=[0 1 1 0 0 ;                 
            0 0 0 0 2 ;
            0 0 0 2 0 ;
            0 0 0 0 1 ;
            0 0 0 0 0 ;];


follower_pos =[x' , y'];
leader_pos  =[x_leader, y_leader];

all_agent = [leader_pos;follower_pos];

obst_pos    = [x_obs', y_obs'];
end_pos    =[220,340 ; -100,0; -140,340; 210,0];




[x1,y1] = draw_circle_f(follower_pos(:,1),follower_pos(:,2),r );
 [xl,yl] = draw_circle_f(leader_pos(:,1),leader_pos(:,2),r_leader);


 [xob, yob, fill_handles] = draw_circle_obs(obst_pos(:,1), obst_pos(:,2), r_obs);

 
[x_end,y_end] = draw_circle_f(end_pos(1),end_pos(2),3);     



map.agentx = x1;
map.agenty = y1;
map.leaderx = xl;
map.leadery = yl;
map.obstaclex = xob;
map.obstacley = yob;
map.obstacleFills = fill_handles;
map.endx = x_end;
map.endy = y_end; 

 markers = {'o'};
 
 colors = {[255/255, 0/255, 0/255], [35/255, 85/255, 206/255], [114/255, 47/255, 16/255], ...
     [30/255, 76/255, 99/255],[255/255, 165/255, 0/255],[114/255, 156/255, 68/255]};




 
     colorss = {[255/255, 0/255, 0/255], [160/255, 160/255, 160/255], [114/255, 47/255, 16/255], ...
      [30/255, 76/255, 99/255],[255/255, 165/255, 0/255],[114/255, 156/255, 68/255]};
  


    for j = 1:num_leader
        hold on
        plot_leader = plot(map.leaderx{j},map.leadery{j},'r' ,LineWidth=2);
        robot_struct.rleader(j) = plot_leader;
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

   


title(' Agent-Agent Avoidance via APF method', 'FontSize', 17);  % You can also set the font size
xlabel('X position ', 'FontSize', 17);
ylabel('Y position ', 'FontSize', 17);


legend_labels = {'Agent', 'Obs'};


legend_handles = gobjects(1 , 1); % +1 for the Obstacle

hold on; 
for i = 1
   
    legend_handles(i) = plot(NaN, NaN, 'MarkerEdgeColor', 'k', 'Color', colorss{i}, 'MarkerSize', 10,LineWidth=2);
end


hLegend = legend(legend_handles, legend_labels);

set(hLegend, 'FontSize', 12, 'Box', 'on');
hold off; 


agent_trajectories(num_agents) = struct('x', [], 'y', []);





  
xlim([-250, 250]);
ylim([250, 350]); 


set(gcf, 'Position',  [400, 400, 500, 500])  


axis equal;

grid on;
box on;


 

    steps = 1;
   
    F_rep_obs = zeros(num_obstacles,2);

    F_rep_aget = zeros(num_agents,2);

    form_goal = e_form*ones(num_agents-1,1)';

    f_goal = 1; 
    
   
    A = ((abs(norm(leader_pos)-norm(end_pos)) > f_goal)); %Check if formation is at goal
    
   

    distance_threshold = 1.0;
    
  
    error_history = zeros(num_agents-1, 2800); % 假设最大步数为640



groups = {[2:7,2]};
agent_lines = gobjects(sum(cellfun(@numel, groups)) - length(groups), 1); % 根据实际需要连线的数量初始化

line_index = 1; 

hold on


    %%

    trajectory = cell(num_agents, 1); 
for i = 1:num_agents
    trajectory{i} = [all_agent(i, :)];
end


    %%
    while A ~=0 

     



for i = 1:num_leader
    F_attr_leader = AttractiveF(leader_pos(i,:) ,end_pos(i,:),0.05,3); 
    repulsion_force_leader = zeros(1, 2);

 
    for k = 1:num_leader
        if i ~= k 
           repulsion_force_leader_agent  = compute_repulsion_new(leader_pos(i,:), leader_pos(k,:), repulsion_max_agent, safe_dis_agent, k_repulsion_agent, d_max_repulsion_agent, r_leader(i), r_leader(k));
           repulsion_force_leader = repulsion_force_leader + repulsion_force_leader_agent;
        end
    end

    velocity_leader = F_attr_leader + repulsion_force_leader ;

    leader_pos(i,:) = leader_pos(i,:) + velocity_leader;
    
     robot_struct.rleader(i).XData = robot_struct.rleader(i).XData+ velocity_leader(1);
     robot_struct.rleader(i).YData = robot_struct.rleader(i).YData+ velocity_leader(2);


end


    %%

     
for i = 1:num_leader
    if isempty(trajectory{i})
        trajectory{i} = leader_pos(i, :); 
    else
        trajectory{i} = [trajectory{i}; leader_pos(i, :)]; 
    end
end

        






%% 画线
 all_agent = [leader_pos; follower_pos];



       
        hold on;
        for i = 1:num_leader
            color = colors{mod(i-1, length(colors)) + 1};
           plot(trajectory{i}(:, 1), trajectory{i}(:, 2), 'Color', color, 'LineWidth', 1.5,'LineStyle',":",'HandleVisibility', 'off');
        end
        hold off;
  

%   
         pause(0.05)
          drawnow
      
        A = norm(leader_pos - end_pos) >= distance_threshold;
    
        steps = steps + 1;
        if steps >= 240
            break
        end

    end


   %  close(v);
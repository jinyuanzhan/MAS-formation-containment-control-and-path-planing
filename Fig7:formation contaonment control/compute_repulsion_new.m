
function [repulsion] = compute_repulsion_new(robot_pose, obs_pose, repulsion_max, d_safe, k_rep, d_max,radius_agent,obs_radius)

    repulsion = [0,0];
    d_io_eff = norm(robot_pose - obs_pose) - (radius_agent+ obs_radius+ (d_safe));

    if d_io_eff < d_max
       
         force_magnitude = ( (1/d_io_eff) - (1/d_max)) *(1/(d_io_eff)^2);

         direction_vector = (robot_pose - obs_pose) / norm(robot_pose - obs_pose);
         
         repulsion_component = force_magnitude * direction_vector*k_rep;
        
      
         repulsion_component = min(repulsion_component, repulsion_max);
         repulsion_component = max(repulsion_component, -repulsion_max);

       
        repulsion_component_x = sign(repulsion_component(1)) * min(abs(repulsion_component(1)), repulsion_max);
        repulsion_component_y = sign(repulsion_component(2)) * min(abs(repulsion_component(2)), repulsion_max);
        repulsion_component = [repulsion_component_x, repulsion_component_y];

    
        repulsion = repulsion + repulsion_component;
    else
        repulsion = [0,0];
    end
end



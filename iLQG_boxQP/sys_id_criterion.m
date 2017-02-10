% Current code assumes that state estimates are coming in at around the
% same rate or faster than state estimates. 

function [cost] = sys_id_criterion(params)


    global msgs_state msgs_cmd;
    global params_to_optimize;

    params = params.*params_to_optimize;
    
    cost = 0;
    num_states = size(msgs_state, 1);

    current_cmd_index = 1;
    
    for i = 2:num_states
        % Read current state
        [x_observed, t_observed] = getStateAndTimeAt(i);
        [x_past, t_past] = getStateAndTimeAt(i-1);
        
        % Find control input closest state time in past
        if(t_cmd > t_past)
            continue
        end
        while(t_cmd < t_past)
            t_cmd = double(msgs_cmd{current_cmd_index,1}.Header.Stamp.Nsec)/10e9;
            current_cmd_index = current_cmd_index + 1;
        end
        current_cmd_index = current_cmd_index - 1;
        if ((t_past-t_cmd) > 1.0)
            % If it's been a long time since control input, disregard
            continue
        end
        
        % Assume constant control input between past state and current.
        % Not necessarily true; input may have changed in between
        % Check for this? 
        dt = t_observed - t_past;
        u = [msgs_cmd{current_cmd_index,1}.Linear.X, ...
             msgs_cmd{current_cmd_index,1}.Angular.Z];
     
        % Compare observed state to expected state
        % Make sure to check expected state as function of past input.
        x_expected = dynamics_finite(x_past, u, dt, params);
        
        cost = cost + metric(x_expected, x_observed);
    end
end

function [dist] = metric(x1, x2)
    dist = pdist2(x1, x2, 'euclidean');
end

function [state, t] = getStateAndTimeAt(index)
        t = double(msgs_state{index,1}.Header.Stamp.Nsec)/10e9;
        
        pos = [msgs_state{index,1}.Pose.Pose.Position.X, ...
               msgs_state{index,1}.Pose.Pose.Position.Y];
        
        vel = [msgs_state{index,1}.Twist.Twist.Linear.X, ...
               msgs_state{index,1}.Twist.Twist.Linear.Y];
        
        q = [msgs_state{index,1}.Pose.Pose.Orientation.W, ...
             msgs_state{index,1}.Pose.Pose.Orientation.X, ...
             msgs_state{index,1}.Pose.Pose.Orientation.Y, ...
             msgs_state{index,1}.Pose.Pose.Orientation.Z];
        eul = quat2eul(q);
        theta = eul(1); % default order is zyx
        
        angvel = msgs_state{index,1}.Twist.Twist.Angular.Z;
        
        state = [pos, theta, vel, angvel];

end
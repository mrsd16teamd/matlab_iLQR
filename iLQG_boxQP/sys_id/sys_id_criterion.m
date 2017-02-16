% Current code assumes that state estimates are coming in at around the
% same rate or faster than state estimates. 

function [cost] = sys_id_criterion(params)

    global stateData;
    global params_to_optimize default_params;

    params = params.*params_to_optimize + default_params.*(1-params_to_optimize);
    
    cost = 0;
    num_states = size(stateData, 1);
    
    for i = 2:num_states
        past = stateData(i-1);
        now  = stateData(i); 
        
        dt = (now.Header.Stamp.Nsec - past.Header.Stamp.Nsec)*10^(-9);
        
        % Compare observed state to expected state
        % Make sure to check expected state as function of past input.
        x_expected = dynamics_finite(past.X', past.U', dt, params);
        
        cost = cost + metric(x_expected, now.X');
    end
end

function [dist] = metric(x1, x2)
    % Euclidean distance
    diff = x1-x2;
    
    weights = [5, 5, 5, 1, 1, 1]';  
    diff = diff.*weights;
    
    dist = sqrt(diff'* diff);
end
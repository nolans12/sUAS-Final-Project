function [trim_state, control_state, control_trim] = straight_trim(trim_def, aircraft_parameters)
    %% Inputs:
        % trim_def: [Va; flight path angle; height]
        % aircraft_parameters: struct containing aircraft parameters

    %% Outputs:
        % trim_state: [xE; yE; zE; phi, theta, psi, uE, vE, wE, p, q, r]
        % control_state: [delta_e; delta_a; delta_r; delta_t]


    % The goal is to find the trim variables that minimize the cost function
    % We will use fmincon to do this
    trim_var_0 = [0; 0; 0]; % Initial guess for the trim variables

    % Define the cost function
    cost_fun = @(trim_var) straight_cost(trim_var, trim_def, aircraft_parameters);

    [control_trim, cost] = fmincon(cost_fun, trim_var_0);

    % Now get the state from the trim variables
    [trim_state, control_state] = straight_state_from_trim(trim_def, control_trim);
    
end
function cost = straight_cost(trim_vars, trim_def, aircraft_parameters)
    %% Inputs:
        % trim_vars: [alpha_0, elevator_0, throttle_0]
        % trim_def: [Va; flight path angle; height]
        % aircraft_parameters: struct containing aircraft parameters
        % wind_inertial: [u_wind; v_wind; w_wind]

    %% Outputs:
        % cost: scalar cost function value

    % Get the state and control vectors from the trim variables
    [state_trim, control_trim] = straight_state_from_trim(trim_def, trim_vars);

    % Get the height/density
    h = trim_def(3);
    rho = stdatmo(h);

    % We define the cost function to be the magnitude of the body forces and moments:
    [f, m] = AircraftForcesAndMoments(state_trim, control_trim, [0; 0; 0], rho, aircraft_parameters);

    % The cost is the sum of the magnitudes of the forces and moments
    cost = norm(f) + norm(m);

end
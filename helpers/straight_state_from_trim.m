function [state_trim, control_trim] = straight_state_from_trim(trim_def, trim_vars)
    %% Inputs:
        % trim_def: [Va; flight path angle; height]
        % trim_vars: [alpha_0, elevator_0, throttle_0]

    %% Outputs:
        % state_trim: [xE; yE; zE; phi, theta, psi, uE, vE, wE, p, q, r]
        % control_trim: [delta_e; delta_a; delta_r; delta_t]

    % Unpack the trim definition
    Va = trim_def(1);
    gamma = trim_def(2);
    h = trim_def(3);

    % Unpack the trim variables
    alpha_0 = trim_vars(1);
    elevator_0 = trim_vars(2);
    throttle_0 = trim_vars(3);
    beta_0 = 0; % for level flight

    % Calculate theta, uE, wE from the inputs
    zE_trim = -h;
    theta_trim = gamma + alpha_0;
    uE_trim = Va * cos(alpha_0) * cos(beta_0);
    wE_trim = Va * sin(alpha_0) * cos(beta_0);

    state_trim = [0; 0; zE_trim; 0; theta_trim; 0; uE_trim; 0; wE_trim; 0; 0; 0];
    control_trim = [elevator_0; 0; 0; throttle_0];

end

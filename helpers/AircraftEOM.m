function [xdot] = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
    % Calculate xdot, the derivative of the state.

    % First, extract the current state. 

    xE = aircraft_state(1);
    yE = aircraft_state(2);
    zE = aircraft_state(3);
    height = -zE;

    phi = aircraft_state(4);
    theta = aircraft_state(5);
    psi = aircraft_state(6);

    uE = aircraft_state(7);
    vE = aircraft_state(8);
    wE = aircraft_state(9);

    p = aircraft_state(10);
    q = aircraft_state(11);
    r = aircraft_state(12);

    density = stdatmo(height);

    c = aircraft_parameters;

    I = [c.Ix, 0, -c.Ixz;
         0, c.Iy, 0;
         -c.Ixz, 0, c.Iz];

    % Get the forces
    [forces, moments] = AircraftForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

    % Now do the Equations of Motions:

    %% p_dot_e = R_e_b * V_e_b;

    p_dot_e = TransformFromBodyToInertial([uE; vE; wE], [phi; theta; psi]);

    xE_dot = p_dot_e(1);
    yE_dot = p_dot_e(2);
    zE_dot = p_dot_e(3);

    %% o_dot_b_e = T * omega_b;

    phi_dot = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot = q * sin(phi) / cos(theta) + r * cos(phi) / cos(theta);

    %% V_dot_b_e = -omega_mat * V_b_e + F_b / m;

    u_dot_e = r * vE - q * wE + forces(1) / c.m;
    v_dot_e = p * wE - r * uE + forces(2) / c.m;
    w_dot_e = q * uE - p * vE + forces(3) / c.m;

    %% omega_dot_b = I^-1 * (-omega_mat * I * omega_b + G_b);

    w_dot_b = inv(I) * (-cross([p; q; r], I * [p; q; r]) + moments);

    p_dot = w_dot_b(1);
    q_dot = w_dot_b(2);
    r_dot = w_dot_b(3);

    % Thus state is [p_dot_e; o_dot_b_e; V_dot_b_e; omega_dot_b]
    xdot = [xE_dot; yE_dot; zE_dot; phi_dot; theta_dot; psi_dot; u_dot_e; v_dot_e; w_dot_e; p_dot; q_dot; r_dot];

end
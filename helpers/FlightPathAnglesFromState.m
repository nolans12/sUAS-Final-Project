function [flight_angles] = FlightPathAnglesFromState(aircraft_state)
    %% Inputs:
        % aircraft_state: [xE; yE; zE; phi, theta, psi, uE, vE, wE, p, q, r]

    %% Outputs:
        % flight_angles: [Vg; course angle, flight path angle]

    % Extract the state
    xE = aircraft_state(1);
    yE = aircraft_state(2);
    zE = aircraft_state(3);
    phi = aircraft_state(4);
    theta = aircraft_state(5);
    psi = aircraft_state(6);
    uE_B = aircraft_state(7);
    vE_B = aircraft_state(8);
    wE_B = aircraft_state(9);
    p = aircraft_state(10);
    q = aircraft_state(11);
    r = aircraft_state(12);

    % % % Get the groudn speed
    % % Vg = sqrt(uE_B^2 + vE_B^2 + wE_B^2);

    % Translate the velocitys into intertial frame
    V_E_E = TransformFromBodyToInertial([uE_B; vE_B; wE_B], [phi; theta; psi]);

    uE_E = V_E_E(1);
    vE_E = V_E_E(2);
    wE_E = V_E_E(3);

    V_g = norm(V_E_E);

    gamma = -asin(wE_E/ V_g);
    chi = atan2(vE_E , uE_E);

    flight_angles = [V_g; chi; gamma];

    % % Now, we know h_dot = Vg*sin(gamma), thus, gamma = asin(-wE/Vg)
    % gamma = asin(-wE_E / Vg);
    % 
    % % Now, to get chi, cos(chi) = uE_E / Vg
    % chi = atan2(vE_E, uE_E);
    % 
    % % Now, we have the flight path angles
    % flight_angles = [Vg; chi; gamma];

end
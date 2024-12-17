function [aero_forces, aero_moments] = AeroForcesAndMoments_BodyState_WindCoeffs(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters)
% This function takes in the state, control, wind, density, and parameters
% and returns the aero force and moment acting on the aircraft in body
% coordinates.
% For this func the PROPULSIVE FORCE IS CONSIDERED as part of the aero force
% and it does NOT INCLUDE WEIGHT.
% Moment includes aero and prop moments.

% Output is aero_force = [X; Y; Z] and aero_moment = [L; M; N].

    % Extract the state:

    xE = aircraft_state(1);
    yE = aircraft_state(2);
    zE = aircraft_state(3);

    phi = aircraft_state(4);
    theta = aircraft_state(5);
    psi = aircraft_state(6);

    uE = aircraft_state(7);
    vE = aircraft_state(8);
    wE = aircraft_state(9);

    p = aircraft_state(10);
    q = aircraft_state(11);
    r = aircraft_state(12);

    % Extract the control

    del_e = aircraft_surfaces(1);
    del_a = aircraft_surfaces(2);
    del_r = aircraft_surfaces(3);
    del_t = aircraft_surfaces(4);

    % Rename the constant so its less words

    c = aircraft_parameters;

    % Now, get the critical constants

    V_e_b = [uE; vE; wE]; % Inertial rel to body
    % V = V_e_b - wind_inertial; % Air relative vel vec
    w_body = TransformFromInertialToBody(wind_inertial, [phi, theta, psi]);

    V = V_e_b - w_body; % Air relative vel vec


    u = V(1); 
    v = V(2); 
    w = V(3);
    V_a = norm(V); % Air speed

    alpha = atan( w / u ); % AoA
    beta = asin( v / V_a ); % Sideslip

    Q = 0.5 * density * V_a^2; % Dynamic pres

    p_hat = p * c.b / (2 * V_a);
    q_hat = q * c.c / (2 * V_a);
    r_hat = r * c.b / (2 * V_a);

    % Now can get lift and drag

    C_L = c.CL0 + c.CLalpha * alpha + c.CLq * q_hat + c.CLde * del_e;
    C_D = c.CDmin + c.K * (C_L - c.CLmin)^2;

    % Now get the coefficients

    C_X = -cos(alpha) * C_D + sin(alpha) * C_L;
    C_Y = c.CY0 + c.CYbeta * beta + c.CYp * p_hat + c.CYr * r_hat + c.CYda * del_a + c.CYdr * del_r; 
    C_Z = -sin(alpha) * C_D - cos(alpha) * C_L;
    C_L = c.Cl0 + c.Clbeta * beta + c.Clp * p_hat + c.Clr * r_hat + c.Clda * del_a + c.Cldr * del_r;
    C_M = c.Cm0 + c.Cmalpha * alpha + c.Cmq * q_hat + c.Cmde * del_e;
    C_N = c.Cn0 + c.Cnbeta * beta + c.Cnp * p_hat + c.Cnr * r_hat + c.Cnda * del_a + c.Cndr * del_r;

    % calculate coeff of thrust

    C_T = 2 * (c.Sprop / c.S) * c.Cprop * (del_t / (V_a*V_a)) * (V_a + del_t * (c.kmotor - V_a)) * (c.kmotor - V_a);

    aero_forces = c.S * Q * [C_X + C_T; C_Y; C_Z];
    aero_moments = [c.b * c.S * Q * C_L ; c.c * c.S * Q * C_M ; c.b * c.S * Q * C_N];
end


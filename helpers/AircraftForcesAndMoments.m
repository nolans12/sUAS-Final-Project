function [aircraft_forces, aircraft_moments] = AircraftForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters)
% The total force includes the aero force, propulsive force, and weight.

    % First, get the aero force:
    [aero_forces, aero_moments] = AeroForcesAndMoments_BodyState_WindCoeffs(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

    % Now, calculate the gravity forces
    
    c = aircraft_parameters;
    g = 9.81;

    phi = aircraft_state(4);
    theta = aircraft_state(5);
    psi = aircraft_state(6);

    weight_e = [0; 0; c.m * g];
    weight_b = TransformFromInertialToBody(weight_e, [phi; theta; psi]);

    aircraft_forces = aero_forces + weight_b;
    aircraft_moments = aero_moments;
end


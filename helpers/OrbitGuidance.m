function control_objectives = OrbitGuidance(aircraft_position, orbit_speed, orbit_radius, orbit_center, orbit_flag, orbit_gains)
% OrbitGuidance - Generate guidance commands for orbital flight
%
% Syntax:
%   control_objectives = OrbitGuidance(aircraft_position, orbit_speed, orbit_radius, orbit_center, orbit_flag, orbit_gains)
%
% Inputs:
%   aircraft_position - 3x1 vector of aircraft position [x; y; z] (m)s
%   orbit_speed       - Desired orbital speed (m/s)
%   orbit_radius      - Desired orbit radius (m)
%   orbit_center      - 3x1 vector of orbit center position [x; y; z] (m)
%   orbit_flag        - Flag indicating the direction of orbit, either 1 for CW or -1 for CCW
%   orbit_gains       - Struct containing orbit control gains
%                       .kr - Gain for radial control
%                       .kz - Gain for altitude control
%
% Outputs:
%   control_objectives - 5x1 vector of control objectives
%                        (1) h_c       - Commanded altitude (m)
%                        (2) h_dot_c   - Commanded altitude rate (m/s)
%                        (3) chi_c     - Commanded course angle (rad)
%                        (4) chi_dot_ff - Commanded course rate (feedforward) (rad/s)
%                        (5) Va_c      - Commanded airspeed (m/s)
%
% Description:
%   This function generates guidance commands for orbital flight based on
%   the current aircraft position and desired orbit parameters. It calculates
%   the appropriate altitude, course, and airspeed commands to maintain the
%   desired circular orbit.

    c_n = orbit_center(1);
    c_e = orbit_center(2);
    c_d = orbit_center(3);

    p_n = aircraft_position(1);
    p_e = aircraft_position(2);

    % Calculate the radial distance from the aircraft to the orbit center
    d = sqrt((p_n - c_n)^2 + (p_e - c_e)^2);

    % Calculate phi
    phi = atan2(p_e - c_e , p_n - c_n) + 2*pi;

    % Mod phi to be between -pi and pi
    phi = mod(phi, 2*pi);

    % Now compute the commanded course angle using eq 10.13

    chi_0 = phi + orbit_flag * (pi/2);
    chi_c = chi_0 + orbit_flag * atan2(orbit_gains.kr * (d - orbit_radius), orbit_radius);

    % chi_dot_ff = orbit_flag * orbit_speed / orbit_radius;
    chi_dot_ff = 0;

    % Height commands
    h_c = -c_d;
    
    % h_dot_c = orbit_gains.kz * (h_c - aircraft_state(3));
    h_dot_c = 0;

    % Now compute the commanded airspeed using eq 10.15

    Va_c = orbit_speed;

    % Construct control objectives vector
    control_objectives = [h_c; h_dot_c; chi_c; chi_dot_ff; Va_c];

end
    
    
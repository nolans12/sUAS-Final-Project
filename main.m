% Nolan Stevenson
% ASEN 5128
%
% Main file for the final project
%

close all; % <========= Comment out this line and you can run this file multiple times and plot results together
clear all; 
clc;

%%% Aircraft parameters
ttwistor
sensor_params = SensorParametersTtwistor(aircraft_parameters);
% sensor_params.sig_gps = sensor_params.sig_gps ./ 5000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Load control gains
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

gains_file = 'ttwistor_gains_slc';
fprintf(1, '\n ==================================== \nAUTOPILOT: Simple SLC\n \n')

load(gains_file)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Set the wind gradient function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

direction = [1; 1; 0];
direction = direction / norm(direction);
h_ground = 1655;
wind_func = @(h, a, b) [direction(1) * a * log((h - h_ground) + b) .* ones(size(h));
                  direction(2) * a * log((h - h_ground) + b) .* ones(size(h));
                  zeros(size(h))];

% wind_func = @(h, a, b) [1 .* ones(size(h)); 1 .* ones(size(h)); zeros(size(h))];

a = 1.5;
b = 10;


% Make a plot showing the wind profile as a function of height abover ground

h_ex = h_ground:1:h_ground + 250;
wind_ex = wind_func(h_ex, a, b);
wind_mag_ex = zeros(size(h_ex));
for i = 1:length(h_ex)
    wind_mag_ex(i) = norm(wind_ex(:,i));
end

figure(10);
set(gcf, 'Position', [100 100 800 600]); % Make figure larger
plot(wind_mag_ex, h_ex - h_ground, 'LineWidth', 3);
grid on;
hold on;

ax = gca;
ax.FontSize = 14;
ax.LineWidth = 1.5;
ax.XLim = [min(wind_mag_ex)-0.5 max(wind_mag_ex)+0.5];
ax.YLim = [min(h_ex-h_ground)-5 max(h_ex-h_ground)+5];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Set simulation and control parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ts = sensor_params.Ts_imu; % which should be 0.1 sec;
% sensor_params.Ts_gps = Ts; % <============================ Uncomment this line if you want GPS to run as fast as the IMU

Tfinal = 50;
control_gain_struct.Ts = Ts;

% Iterate at control sample time
n_ind = Tfinal / Ts;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Define and Initialize Aircraft Structures
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

num_aircraft = 1; % Number of aircraft
h_offset = 25; % diff b/w aircraft

% Define common parameters
V_trim = 18;
h_trim = h_ground + h_offset;
gamma_trim = 0;
trim_definition = [V_trim; gamma_trim; h_trim];

% Define line-following parameters
Va = 18;
direction = [1; 1; -0.5]; % in inertial [x, y, z]
v0 = Va * direction / norm(direction);
lookahead_distance = 10000*Va;

% Initialize aircraft structures
for ac_id = 1:num_aircraft
    h_ac = h_trim + h_offset * (ac_id - 1);
    ac(ac_id).p0 = [0; 0; -h_ac]; % Offset initial position
    temp_trim = trim_definition;
    temp_trim(3) = h_ac;
    ac(ac_id).v0 = v0;
    ac(ac_id).lookahead_distance = lookahead_distance;
    ac(ac_id).color = ['r', 'g', 'b']; % Assign colors for plotting

    % Trim and initialize state
    [aircraft_state_trim, control_input_trim, ~] = straight_trim(temp_trim, aircraft_parameters);
    ac(ac_id).aircraft_state_true(:, 1) = aircraft_state_trim;
    ac(ac_id).control_array(:, 1) = control_input_trim;
    ac(ac_id).wind_array_true(:, 1) = wind_func(h_ac, a, b);
    ac(ac_id).time_iter(1) = 0;
    ac(ac_id).gps_globals = struct('phat', [], 'qhat', [], 'rhat', [], ...
                                    'press_stat', [], 'press_dyn', [], ...
                                    'phi_hat', [], 'theta_hat', [], 'P_est', [], ...
                                    'xhat_gps', [], 'P_gps', []);
    ac(ac_id).gps_noise = [];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Simulation Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a_guess = 0;
a_step = 0.01;

b_guess = 0;
b_step = 2.5;

for i = 1:n_ind
    TSPAN = Ts * [i-1 i];

    wind_meas = []; % Includes v and h for each aircraft

    % Process each aircraft independently
    for ac_id = 1:num_aircraft
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% GET THE WIND
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        wind_inertial = wind_func(-ac(ac_id).aircraft_state_true(3, i), a, b);
        ac(ac_id).wind_array_true(:, i) = wind_inertial; 

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Sensor measurements - Based on true position
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        [gps_sensor, ac(ac_id).gps_noise] = GPSSensor(ac(ac_id).aircraft_state_true(:, i), sensor_params, ac(ac_id).gps_noise);
        inertial_sensors = InertialSensors(ac(ac_id).aircraft_state_true(:, i), ...
                                           ac(ac_id).control_array(:, i), ...
                                           ac(ac_id).wind_array_true(:, i), ...
                                           aircraft_parameters, sensor_params);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Estimator
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 
        % [aircraft_state_est, wind_inertial_est, ac(ac_id).gps_globals] = EstimatorAttitudeGPSSmoothing(ac(ac_id).time_iter(i), ...
        %                                                                         gps_sensor, ...
        %                                                                         inertial_sensors, ...
        %                                                                         sensor_params, ...
        %                                                                         ac(ac_id).gps_globals, wind_inertial);


        [aircraft_state_est, wind_inertial_est, ~, ~, ~, ac(ac_id).gps_globals] = EstimatorAttitudeGPSSmoothing(ac(ac_id).time_iter(i), ...
                                                                                gps_sensor, ...
                                                                                inertial_sensors, ...
                                                                                sensor_params, ...
                                                                                ac(ac_id).gps_globals);

        % Store wind estimation error and height
        % ac(ac_id).wind_error(:,i) = wind_inertial_est - wind_inertial;
        % fprintf('Aircraft %d - Height: %.2f m, Wind Error: [%.2f, %.2f, %.2f] m/s\n', ...
        %     ac_id, -aircraft_state_est(3) - h_ground, ...
        %     wind_inertial_est(1)-wind_inertial(1), ...
        %     wind_inertial_est(2)-wind_inertial(2), ...
        %     wind_inertial_est(3)-wind_inertial(3));


        ac(ac_id).aircraft_state_est(:, i) = aircraft_state_est;

        % Estimate wind angles
        wind_body_est = TransformFromInertialToBody(wind_inertial_est, aircraft_state_est(4:6));
        air_rel_est = aircraft_state_est(7:9) - wind_body_est;
        ac(ac_id).wind_angles_est(:, i) = AirRelativeVelocityVectorToWindAngles(air_rel_est);

        % Store the measurement for this aircraft
        wind_meas = [wind_meas, [norm(wind_inertial_est); -ac(ac_id).aircraft_state_est(3, i)]];

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Line following guidance - Based on estimated
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        current_pos = [aircraft_state_est(1); aircraft_state_est(2); aircraft_state_est(3)];
        [vec_follow, waypoint_follow] = lineFollowVector(0, current_pos, ac(ac_id).p0, ac(ac_id).v0, ac(ac_id).lookahead_distance);
        command_state = calcCommandFromVec(waypoint_follow, vec_follow);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Autopilot - Decide control based on estimated
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        [control_out, ~] = SimpleSLCAutopilot(Ts * (i-1), ...
                                              aircraft_state_est, ...
                                              ac(ac_id).wind_angles_est(:, i), ...
                                              command_state, ...
                                              control_gain_struct);
    
        ac(ac_id).control_array(:, i+1) = control_out;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Aircraft dynamics - Based on actual state and wind
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        [TOUT, YOUT] = ode45(@(t, y) AircraftEOM(t, y, control_out, ...
                                                   ac(ac_id).wind_array_true(:, i), ...
                                                   aircraft_parameters), ...
                              TSPAN, ac(ac_id).aircraft_state_true(:, i), []);

        ac(ac_id).aircraft_state_true(:, i+1) = YOUT(end, :)';
        ac(ac_id).time_iter(i+1) = TOUT(end);
    end

   % Online gradient descent loop
    for i = 1:width(wind_meas)
        v_i = wind_meas(1, i);
        h_i = wind_meas(2, i) - h_ground;

        % Compute gradients
        if h_i + b_guess > 0
            loss_a = -(v_i - a_guess * log(h_i + b_guess)) * log(h_i + b_guess);
            loss_b = -(v_i - a_guess * log(h_i + b_guess)) * (a_guess / (h_i + b_guess));
        else
            % Skip update if infeasible (optional)
            fprintf('Skipped update at iteration %d due to infeasibility\n', i);
            continue;
        end

        % Gradient update
        a_guess = a_guess - a_step * loss_a;
        b_guess = b_guess - b_step * loss_b;

        % Project b_guess to ensure feasibility (h_i + b_guess > 0)
        b_guess = max(b_guess, -h_i + 1e-6);
    end

    % Compute the new wind profile estimate
    wind_est = wind_func(h_ex, a_guess, b_guess);
    wind_mag_est = zeros(size(h_ex));
    for i = 1:length(h_ex)
        wind_mag_est(i) = norm(real(wind_est(:,i)));
    end

    % Get current figure and clear it
    figure(10);
    clf;

    % Plot true wind profile
    plot(wind_mag_ex, h_ex - h_ground, 'LineWidth', 3, 'DisplayName', 'True Wind Profile');
    grid on;
    hold on;

    % Plot estimated wind profile
    plot(wind_mag_est, h_ex - h_ground, 'LineWidth', 2, 'DisplayName', sprintf('Estimate (a=%.2f, b=%.2f)', a_guess, b_guess));

    % Plot measurement points
    scatter(wind_meas(1,:), wind_meas(2,:) - h_ground, 100, 'k*', 'LineWidth', 2, 'DisplayName', 'Measurements', 'HandleVisibility', 'on');

    % Set axes properties
    ax = gca;
    ax.FontSize = 14;
    ax.LineWidth = 1.5;
    ax.XLim = [min(wind_mag_ex)-0.5 max(wind_mag_ex)+0.5];
    ax.YLim = [min(h_ex-h_ground)-5 max(h_ex-h_ground)+5];

    xlabel('Magnitude of Wind (m/s)', 'FontSize', 16);
    ylabel('Height above ground (m)', 'FontSize', 16);
    title(sprintf('Wind Profile vs Height, True: (a = %.2f, b = %.2f) (%d CLIMBING Aircraft, t=%.1f seconds)', a, b, num_aircraft, TSPAN(1)), 'FontSize', 18);
    legend('Location', 'best');

    if TSPAN(1) == 5
        test = 1
    end


    if TSPAN(1) == 20
        test = 1
    end


    if TSPAN(1) == 40
        test = 1
    end
    drawnow;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for ac_id = 1:num_aircraft
    PlotSimulation(ac(ac_id).time_iter, ac(ac_id).aircraft_state_true, ac(ac_id).control_array, ac(ac_id).color(ac_id));
end

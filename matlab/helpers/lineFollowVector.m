function [dXdt, waypoint_follow] = lineFollowVector(t, X, line_initial, line_velocity, lookahead_distance)

    %% Used for ode45 to output the deriative of the state 

    % X:
    %   X(1) = p_n
    %   X(2) = p_e
    %   X(3) = p_d (which is -h)

    % line_initial:
    %   line_initial(1) = line_n
    %   line_initial(2) = line_e
    %   line_initial(3) = line_d

    % line_velocity:
    %   Has magnitude equal to Va
    %   Is a unit vector in the direction of the line velocity

    % lookahead_distance:
    %   Distance ahead of the line to look for the next point

    Va = norm(line_velocity);
    line_unit = line_velocity / Va;

    % First, get the relative position vector
    p_rel = X(1:3) - line_initial;

    % Now, find the projection of p_rel onto the line velocity vector
    p_proj = dot(p_rel, line_unit);

    % Now, take initial point + p_proj + lookahead distance * line_velocity
    p_target = line_initial + (p_proj + lookahead_distance) * line_unit;

    % Now get the vector from the current position to the target point
    p_error = p_target - X(1:3);

    % Thus, velocity vector is just p_error unit vector * Va
    dXdt = (p_error / norm(p_error)) * Va;

    waypoint_follow = p_target;

end


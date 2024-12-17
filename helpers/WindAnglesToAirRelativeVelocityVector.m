function velocity_body = WindAnglesToAirRelativeVelocityVector(wind_angles)
% Nolan Stevenson, 9-4-24

% Calculate the aircraft air relative velocity vector in body coordinates
% from the airspeed, sideslip, and AoA. [V_a; beta; alpha] input.
% Return is [u; v; w]

    V_a = wind_angles(:,1);
    beta = wind_angles(:,2);
    alpha = wind_angles(:,3);

    % Create transformation matrix for each data point
    n = length(V_a);
    velocity_body = zeros(3,n);
    
    for i = 1:n
        mat = [cos(alpha(i))*cos(beta(i)); 
              sin(beta(i)); 
              sin(alpha(i))*cos(beta(i))];
              
        velocity_body(:,i) = V_a(i) * mat;
    end

    velocity_body = velocity_body';

end

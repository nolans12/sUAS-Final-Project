function [wind_angles] = AirRelativeVelocityVectorToWindAngles(velocity_body)
% Nolan Stevenson, 9-4-24

% Given the air relative velocity vector in body coordinate v_b, this
% function returns the wind angles in the col vec [V_a; beta; alpha]

    % The input v_b is [u; v; w]

    u = velocity_body(1);
    v = velocity_body(2);
    w = velocity_body(3);
   
    V_a = sqrt(u^2 + v^2 + w^2); 

    beta = asin(v / V_a); % rad

    % alpha = atan(w / u); % rad
    alpha = atan2(w , u); % rad

    wind_angles = [V_a; beta; alpha];
    
end


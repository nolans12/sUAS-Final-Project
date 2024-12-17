function R = Rotation321(euler_angles)
% Nolan Stevenson, 9-4-24

% Does the 3-2-1 rotation matrix given the Euler angles,
% Euler angles specified with [phi, theta, psi]

    phi = euler_angles(1); % rad, roll
    theta = euler_angles(2); % rad, pitch
    psi = euler_angles(3); % rad, yaw

    R1 = [1, 0,         0;
          0, cos(phi),  sin(phi);
          0, -sin(phi), cos(phi)];

    R2 = [cos(theta), 0, -sin(theta);
          0,          1, 0;
          sin(theta), 0, cos(theta)];

    R3 = [cos(psi),  sin(psi), 0;
          -sin(psi), cos(psi), 0;
          0,         0,        1];

    R = R1 * R2 * R3;

end


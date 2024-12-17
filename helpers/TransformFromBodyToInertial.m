function vector_inertial = TransformFromBodyToInertial(vector_body, euler_angles)
% Nolan Stevenson, 9-4-24

% From a vector given in body coords, determine the components in inertial
% coordinates

    % First calculate the rotation matrix from euler angles
    R_e2b = Rotation321(euler_angles);

    % Now transpose to get body to inertial
    R_b2e = R_e2b';

    % Now, we have the rotation matrix from inertial to body
    vector_inertial = R_b2e * vector_body;

end




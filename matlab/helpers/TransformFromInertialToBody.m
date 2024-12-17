function vector_body = TransformFromInertialToBody(vector_inertial, euler_angles)
% Nolan Stevenson, 9-4-24

% From a vector given in inertial coords, determine the components in body
% coordinates

    % First calculate the rotation matrix from euler angles
    R_e2b = Rotation321(euler_angles);

    % Now, we have the rotation matrix from inertial to body
    vector_body = R_e2b * vector_inertial;

end


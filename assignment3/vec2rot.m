function C = vec2rot(k)
% Ensure k is a column vector
k = k(:);

% Compute the rotation angle (phi) as the norm of k
phi = norm(k);

% Handle the zero rotation case
if phi < 1e-8
    C = eye(3);
    return;
end

% Compute the unit rotation axis (a)
a = k / phi;  % Normalize k to get the axis

% Skew-symmetric matrix of a
a_cross = [   0,   -a(3),  a(2);
    a(3),    0,  -a(1);
    -a(2),  a(1),    0];

% Compute the rotation matrix using the formula
C = cos(phi) * eye(3) + (1 - cos(phi)) * (a * a') - sin(phi) * a_cross;
end

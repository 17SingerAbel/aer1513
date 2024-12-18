function T_inv = inv_SE3(T)
% Computes the inverse of an SE(3) matrix
    R = T(1:3, 1:3);   % Rotation matrix
    t = T(1:3, 4);     % Translation vector
    T_inv = [R', -R' * t;
             0, 0, 0, 1];
end
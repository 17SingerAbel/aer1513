function vec = vee(mat)
if all(size(mat) == [4, 4]) % Case for SE(3)
    % Extract translational and rotational components
    v = mat(1:3, 4);         % Translational part
    w_mat = mat(1:3, 1:3);   % Rotational skew-symmetric part
    
    % Convert skew-symmetric part to a vector
    w = [w_mat(3, 2); w_mat(1, 3); w_mat(2, 1)];
    
    % Construct the 6x1 vector
    vec = [v; w];
    
elseif all(size(mat) == [3, 3]) % Case for SO(3)
    % Convert skew-symmetric part to a vector
    vec = [mat(3, 2); mat(1, 3); mat(2, 1)];
    
else
    error('Input matrix must be 3x3 or 4x4.');
end
end

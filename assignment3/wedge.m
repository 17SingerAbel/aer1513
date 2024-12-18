function mat = wedge(vec)
    if numel(vec) == 6  % Case for SE(3)
        % Extract translational and rotational components
        v = vec(1:3);  % Translational part
        w = vec(4:6);  % Rotational part

        % Create skew-symmetric matrix for w
        w_mat = [  0   -w(3)  w(2);
                   w(3)  0   -w(1);
                  -w(2)  w(1)  0 ];

        % Construct the 4x4 matrix
        mat = [w_mat, v; 0 0 0 0];
        
    elseif numel(vec) == 3  % Case for SO(3)
        % Input vector is a rotational part only
        w = vec;

        % Create skew-symmetric matrix for w
        mat = [  0   -w(3)  w(2);
                 w(3)   0  -w(1);
                -w(2)  w(1)  0 ];
        
    else
        error('Input vector must have 3 or 6 elements.');
    end
end

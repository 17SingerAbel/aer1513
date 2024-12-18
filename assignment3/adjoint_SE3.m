function Ad_AB_inv = adjoint_SE3(T_rel)
    % Step 2: Extract rotational and translational components
    R = T_rel(1:3, 1:3);   % Rotation matrix (3x3)
    t = T_rel(1:3, 4);     % Translation vector (3x1)

    % Step 3: Compute the skew-symmetric matrix of t
    t_hat = [  0     -t(3)   t(2);
                t(3)    0     -t(1);
                -t(2)   t(1)    0  ];

    % Step 4: Construct the Adjoint matrix
    Ad_AB_inv = [R, t_hat * R;
                    zeros(3), R];
    end
    
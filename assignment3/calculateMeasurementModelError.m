function [e_y, G, R_inv] = calculateMeasurementModelError(T_op, y_k_j, ...
    rho_i_pj_i, T_cv, fu, fv, cu, cv, b, y_var, k1, k2, N)
    % Initialize measurement motwist_kel error, Jacobian matrix, and covariance matrix for all timesteps
    K = size(T_op, 3); % Assuming T_op is 3D with the third dimension being timesteps
    e_y = cell(K, 1); % Initialize measurement model error as a cell array, one cell for each timestep
    G = cell(K, 1);   % Initialize Jacobian matrix G as a cell array, one cell for each timestep
    R_inv = cell(K, 1);

    % Camera intrinsic matrix
    M = [fu, 0, cu, 0;
        0, fv, cv, 0;
        fu, 0, cu, -fu * b;
        0, fv, cv, 0];

    for k = k1:k2
        e_y_k = []; % Initialize error for this timestep
        G_k = [];   % Initialize Jacobian for this timestep
        R_k_inv = [];   % Initialize covariance for this timestep

        for j = 1:N % Loop through all landmarks
            if all(y_k_j(:, k, j) ~= -1) % Check if the landmark is observed

                % Coordinate transform:
                p_i_pj_i = [rho_i_pj_i(:, j); 1]; % Compute transformed point z in the camera frame
                p_j_c = T_cv * T_op(:,:,k) * p_i_pj_i;

                % e_y_kj:
                predicted_y_k_j = (M * p_j_c) ./ p_j_c(3); % Predict measurements
                e_y_kj = y_k_j(:, k, j) - predicted_y_k_j; % Calculate the error
                e_y_k = [e_y_k; e_y_kj]; % Stack errors for all observed landmarks

                % Jacobian:
                % z = D' * T_cv * T_op(:,:,k) * p_i_pj_i;
                z = T_cv * T_op(:,:,k) * p_i_pj_i;
                J_g = Jacobian_g(z, fu, fv, b); % Compute the derivative of the observation model g at z
                G_jk = J_g * T_cv * odot_operator(T_op(:,:,k) * p_i_pj_i); % Compute G_jk
                G_k = [G_k; G_jk]; % Stack Jacobians for all observed landmarks

                % Covariance:
                R_k_j_inv = diag((1 ./ y_var)); % Calculate the covariance
                R_k_inv= blkdiag(R_k_inv, R_k_j_inv);
            end
        end

        % Store the errors, Jacobians, and covariance for this timestep
        e_y{k} = e_y_k;
        G{k} = G_k;
        R_inv{k} = R_k_inv;
        % disp(size(G_k));
    end
end

function J_g = Jacobian_g(z, fu, fv, b)
    x = z(1);
    y = z(2);
    z_val = z(3);

    J_g = [fu/z_val, 0, -fu*x/z_val^2, 0;
           0, fv/z_val, -fv*y/z_val^2, 0;
           fu/z_val, 0, -fu*(x-b)/z_val^2, 0;
           0, fv/z_val, -fv*y/z_val^2, 0];
end

% function pixel_coord = estimatePixelLocation(p_i_pj_i, T_cv, T_k, fu, fv, cu, cv, b)
%     % Define D matrix (for projection)
%     D = [1, 0, 0, 0;
%          0, 1, 0, 0;
%          0, 0, 1, 0];

%     % Transform the landmark position to the camera frame
%     z = D * T_cv * T_k * [p_i_pj_i; 1];

%     % Use the camera intrinsic parameters to project onto pixel coordinates
%     pixel_coord = cameraIntrinsic(z, fu, fv, cu, cv, b);
% end

% function pixel_coord = cameraIntrinsic(z, fu, fv, cu, cv, b)
%     % Form the 3D point in homogeneous coordinates
%     p = [z; 1];

%     % Camera intrinsic matrix for the stereo camera
%     M = [fu, 0, cu, 0;
%          0, fv, cv, 0;
%          fu, 0, cu, -fu * b;
%          0, fv, cv, 0];

%     % Project the point to pixel coordinates in the stereo image
%     pixel_coordinates = M * p;
%     pixel_coord = [pixel_coordinates(1) / z(3);  % x-coordinate in left image
%                    pixel_coordinates(2) / z(3);  % y-coordinate in left image
%                    pixel_coordinates(3) / z(3);  % x-coordinate in right image
%                    pixel_coordinates(4) / z(3)]; % y-coordinate in right image
% end

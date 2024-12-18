function [error_v, F, Q_inv] = calculateMotionModelError(T_op, T_op_inv, T_gt, v_vk_vk_i, w_vk_vk_i, v_var, w_var, t, k1, k2)
    % Initialize motion model error, Jacobian matrix, and covariance matrix for all timesteps
    K = size(T_op, 3); % Assuming T_op is 3D with the third dimension being timesteps
    error_v = cell(K, 1);
    F = cell(K, 1);
    Q_inv = cell(K, 1);
    check_T0 = T_gt(:,:,k1);
    start_timestamp = 1215;
    % end_timestamp = 1714;
    T_op(:,:,start_timestamp) = T_gt(:,:,start_timestamp);

    for k = k1:k2
        delta_t = t(k) - t(k-1);
        if k == start_timestamp
            % First input error
            T_diff = check_T0 * T_op_inv(:,:,k1); % check_T0 is taken from the ground truth
            error_v{k} = vee(logm(T_diff)); % vee operator to convert matrix to vector

            F{k} = adjoint_SE3(T_op(:,:,k) * inv_SE3(check_T0)); % Adjoint of the relative transformation -- linearized
        else
            % TODO: -twist_k
            twist_k = [v_vk_vk_i(:,k-1); w_vk_vk_i(:,k-1)];
            xi_k = expm(delta_t * wedge(twist_k));
            error_v{k} = vee(logm(xi_k * T_op(:,:,k-1) * T_op_inv(:,:,k))); % Later input errors

            F{k} = adjoint_SE3(T_op(:,:,k) * T_op_inv(:,:,k-1)); % Adjoint of the relative transformation -- linearized
        end
         % Constructing 6x6 covariance matrix for each timestep
        Q_inv{k} = diag([1./(v_var*delta_t^2); 1./(w_var*delta_t^2)]); 
    end
end

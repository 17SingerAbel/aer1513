function [A, b, H, W_inv, e_stack, e_v_stack, e_y_stack] = calculateH(e_v, e_y, F, G, Q_inv, R_inv, k1, k2)

    % Stack all errors from e_v and e_y and covariance matrix W
    e_v_stack = [];
    e_y_stack = [];
    Q_inv_stack = [];
    R_inv_stack = [];
    empty.error = 0;
    
    for k = k1:k2
        e_v_stack = [e_v_stack; e_v{k}];
        Q_inv_stack = blkdiag(Q_inv_stack, Q_inv{k});
    
        if isempty(e_y{k})
            % e_y_stack = [e_y_stack; zeros(0,1)];
            % R_inv_stack = blkdiag(R_inv_stack, zeros(0,4));
            empty.error = empty.error + 1;
        else
            e_y_stack = [e_y_stack; e_y{k}];
            R_inv_stack = blkdiag(R_inv_stack, R_inv{k});
        end
    end
    
    e_stack = [e_v_stack; e_y_stack];
    W_inv = blkdiag(Q_inv_stack, R_inv_stack);
    e = e_stack;

    
    % Calculate H
    total_e_v_size = size(e_v_stack, 1); % Determine the total size of H
    total_e_y_size = size(e_y_stack, 1);
    H_size = total_e_v_size + total_e_y_size;
    H_v = zeros(total_e_v_size, total_e_v_size); % Preallocate H
    idx = 1; % Initialize index for filling H
    
    % H_v calculation:
    for k = k1:k2
        size_e_v_k = size(e_v{k}, 1); % Size of the current e_v and e_y
        H_v(idx:idx+5, idx:idx+5) = eye(size_e_v_k); % Fill in the blocks for the motion model
        if k > k1
            H_v(idx:idx+5, idx-6:idx-1) = -F{k-1};
        end
        idx = idx + 6; % Update index
    end
    
    total_e_y_size = 0;
    
    for k = k1:k2
        if ~isempty(e_y{k})
            total_e_y_size = total_e_y_size + size(e_y{k}, 1);
        end
    end
    
    idx = 1; % Initialize the index for filling H_y
    empty_meas = 0;
    
    % H_y calculation:
    H_y = zeros(total_e_y_size, 6 * (k2 - k1 + 1)); % Preallocate H_y with the correct size
    
    % Loop through each timestep
    for k = k1:k2
        size_e_y_k = size(e_y{k}, 1);
        if isempty(G{k})
            empty_meas = empty_meas + 1; % If G{k} is empty, no need to fill H_y for this timestep
        else
            col_idx_start = 6 * (k - k1) + 1; % If G{k} is not empty, fill the corresponding part of H_y
            col_idx_end = col_idx_start + 5; % Calculate the column index range for G{k}
            H_y(idx:idx+size_e_y_k-1, col_idx_start:col_idx_end) = G{k};
        end
        idx = idx + size_e_y_k; % Update the row index for the next timestep
    end
    
    H = [H_v; H_y]; % Stack H_v and H_y together
    H_T_W_inv = H' * W_inv;
    
    A = H_T_W_inv * H; % Compute A
    b = H_T_W_inv * e; % Compute b
end
    
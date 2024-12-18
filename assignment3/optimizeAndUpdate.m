function [T_op, delta_x_optimal] = optimizeAndUpdate(A, b, T_op, k1, k2)
    % Optimization Solver using Cholesky Decomposition
    
    % Check if A is symmetric positive definite
    if isequal(A, A') && all(eig(A) > 0)
        % Perform Cholesky decomposition
        L = chol(A, 'lower');
        
        % Solve for delta_x using forward and backward substitution
        y = L \ b;               % Forward substitution
        delta_x_optimal = L' \ y;   % Backward substitution
    else
        % If A is not symmetric positive definite, fall back to another solver
        warning('Matrix A is not symmetric positive definite. Using pinv for solving.');
        delta_x_optimal = pinv(A) * b;
    end
    
    % Update the operating point
    for k = (k1+1):k2
        % Extract perturbation for timestep k
        eps_k_star = delta_x_optimal((k - k1) * 6 + 1 : (k - k1) * 6 + 6);
        
        % Update T_op using the perturbation
        T_op(:,:,k) = expm(wedge(eps_k_star)) * T_op(:,:,k);
    end
    end
    
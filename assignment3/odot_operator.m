function p_odot = odot_operator(p)
% Computes the odot operator as per the given formulation
% Input:
%   p - 4x1 vector, where p = [rho; eta]
%       rho: 3x1 vector
%       eta: scalar
% Output:
%   p_odot - 4x4 matrix computed as:
%            [ eta * I, -rho_hat;
%              0^T,      0^T ]

    % Validate input
    if length(p) ~= 4
        error('Input vector p must be 4x1 with rho (3x1) and eta (scalar).');
    end

    % Extract rho and eta
    rho = p(1:3);  % 3D vector
    eta = p(4);    % Scalar

    % Identity matrix and skew-symmetric matrix
    rho_hat = wedge(rho); % Compute skew-symmetric matrix of rho

    % Construct the odot operator matrix
    p_odot = [eta * ones(3, 3), -rho_hat;
              zeros(1, 3), zeros(1, 3)];
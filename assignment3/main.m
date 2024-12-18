clear all;
clc ;

load('dataset3.mat');
who

k1 = 1215;
k2 = 1714;
maxIterations = 10;


N = size (y_k_j , 3);
K = size (t ,2);
K_total = K;
%
T_i_vk = repmat(eye(4) , [1, 1, K_total]);
T_vk_i = repmat(eye(4) , [1, 1, K_total]);

for k = 1:K
    C_vk_i = vec2rot(theta_vk_i(:,k));
    C_i_vk = C_vk_i.';
    T_i_vk (:,:,k) = [ C_i_vk , r_i_vk_i(:,k); zeros(1, 3), 1];
    T_vk_i (:,:,k) = [ C_vk_i, -C_vk_i * r_i_vk_i(:,k); zeros(1, 3), 1];
end

T_gt = T_vk_i ;
% T_op is T_op_v_i from inertial to vichle frame
T_op = repmat ( eye (4) , [1, 1, K_total ]);
T_op_inv = repmat ( eye (4) , [1, 1, K_total ]);
T_op (:,:, k1) = T_vk_i (:,:, k1);
T_op (: ,: ,1) = T_vk_i (: ,: ,1);
checkT0 = T_vk_i (: ,: ,1);


    %TODO: DIFFERE
for k = k1+1:k2
    delta_t = t(k) - t(k-1);
    twist_k = [-v_vk_vk_i(:,k); -w_vk_vk_i(:,k)]; % input v and w
    xi_k = expm(delta_t * wedge(twist_k)); % added transformation matrix after v, w
    % -- hamburger symbol
    T_op(:,:,k) = xi_k * T_op(:,:,k-1);
    T_op_inv(:,:,k) = inv(T_op(:,:,k));
end


D_t = [
    1 0 0 0;
    0 1 0 0;
    0 0 1 0;
];

T_cv = [C_c_v, -C_c_v * rho_v_c_v; zeros(1, 3), 1];


for iteration = 1: maxIterations
    error_v = cell(K, 1);
    F = cell(K, 1);
    Q = cell(K, 1);
    error_y = cell(K, 1);
    G = cell(K, 1);
    R = cell(K, 1);
    A_matrix = [];
    b_matrix = [];
    delta_x_optimal = [];

    [error_v, F, Q_inv] = calculateMotionModelError(T_op, T_op_inv, T_gt, v_vk_vk_i, w_vk_vk_i, v_var, w_var, t, k1, k2);

    [error_y, G, R_inv] = calculateMeasurementModelError(T_op, y_k_j, rho_i_pj_i, T_cv, fu, fv, cu, cv, b, y_var, k1, k2, N);

    [A_mat, b_mat, H, W_inv, e_stack, e_v_stack, e_y_stack] = calculateH(error_v, error_y, F, G, Q_inv, R_inv, k1, k2);

    [T_op, delta_x_optimal] = optimizeAndUpdate(A_mat, b_mat, T_op, k1, k2);

    eps = norm(delta_x_optimal);
    fprintf ('The current iteration is: %d, and error is at %f ......\ n \n', iteration, eps) ; % Print the current iteration and error
    if eps < 10^ -3
        disp (" The pose estimation successfully converges ! ")
        break ;
    end

end

%% 
plot_error_batch (T_op , T_gt , A_mat , k1 , k2)
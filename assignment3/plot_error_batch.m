function plot_error_batch(T_op, T_gt, A, k1, k2)
    % Initialize error arrays
    rot_err = zeros(k2 - k1 + 1, 3);
    trans_err = zeros(k2 - k1 + 1, 3);
    % A = A(k1*6:(k2+1)*6-1, k1*6:(k2+1)*6-1);

    % Calculate errors
    for k = k1:k2
        C_gt = T_gt(1:3, 1:3, k);
        C_op = T_op(1:3, 1:3, k);

        r_i_gt = -C_gt' * T_gt(1:3, 4, k);
        r_op = -C_op' * T_op(1:3, 4, k);

        rot_err(k-k1+1, :) = vee(eye(3) - C_op * C_gt');
        trans_err(k-k1+1, :) = r_op - r_i_gt;
    end

    % Print average errors
    fprintf('Avg Rot Err: %f\n', mean(abs(rot_err), 'all'));
    fprintf('Avg Trans Err: %f\n', mean(abs(trans_err), 'all'));

    % Calculate variances
    var = diag(inv(A));
    var_tx = var(1:6:end);
    var_ty = var(2:6:end);
    var_tz = var(3:6:end);
    var_rx = var(4:6:end);
    var_ry = var(5:6:end);
    var_rz = var(6:6:end);

    % Time vector
    t = k1:k2;

    % Histogram
    figure;
    histogram(trans_err(:, 1)', 'DisplayName', 'Translation Error in x');
    ylabel('#');
    xlabel('Translation Error in x [m]');

    figure;
    histogram(trans_err(:, 2)', 'DisplayName', 'Translation Error in y');
    ylabel('#');
    xlabel('Translation Error in y [m]');

    figure;
    histogram(trans_err(:, 3)', 'DisplayName', 'Translation Error in z');
    ylabel('#');
    xlabel('Translation Error in z [m]');

    figure;
    histogram(rot_err(:, 1)', 'DisplayName', 'Rotational Error in x');
    ylabel('#');
    xlabel('Rotational Error in x [rad]');

    figure;
    histogram(rot_err(:, 2)', 'DisplayName', 'Rotational Error in y');
    ylabel('#');
    xlabel('Rotational Error in y [rad]');

    figure;
    histogram(rot_err(:, 3)', 'DisplayName', 'Rotational Error in z');
    ylabel('#');
    xlabel('Rotational Error in z [rad]');

    % %% Plotting translational error
    % figure;
    % axis = ["x", "y", "z"];
    % subplot(3, 1, 1);
    % plot(t, trans_err(:, 1), 'DisplayName', 'Translation Error in x', 'LineWidth', 1);
    % hold on;
    % fill([t'; flipud(t')], [+3 * sqrt(var_tx); flipud(-3 * sqrt(var_tx))], 'r', ...
    %     'FaceAlpha', 0.2, 'EdgeColor', 'none');
    % plot(t, +3 * sqrt(var_tx), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope upper bound');
    % plot(t, -3 * sqrt(var_tx), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope lower bound');
    % xlabel('Timestep');
    % ylabel('Error [m]');
    % title(['Translation Error in ' axis(1) ' Axis']);
    % legend;

    % subplot(3, 1, 2);
    % plot(t, trans_err(:, 2), 'DisplayName', 'Translation Error in y', 'LineWidth', 1);
    % hold on;
    % fill([t'; flipud(t')], [+3 * sqrt(var_ty); flipud(-3 * sqrt(var_ty))], 'r', ...
    %     'FaceAlpha', 0.2, 'EdgeColor', 'none');
    % plot(t, +3 * sqrt(var_ty), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope upper bound');
    % plot(t, -3 * sqrt(var_ty), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope lower bound');
    % xlabel('Timestep');
    % ylabel('Error [m]');
    % title(['Translation Error in ' axis(2) ' Axis']);
    % legend;

    % subplot(3, 1, 3);
    % plot(t, trans_err(:, 3), 'DisplayName', 'Translation Error in z', 'LineWidth', 1);
    % hold on;
    % fill([t'; flipud(t')], [+3 * sqrt(var_tz); flipud(-3 * sqrt(var_tz))], 'r', ...
    %     'FaceAlpha', 0.2, 'EdgeColor', 'none');
    % plot(t, +3 * sqrt(var_tz), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope upper bound');
    % plot(t, -3 * sqrt(var_tz), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope lower bound');
    % xlabel('Timestep');
    % ylabel('Error [m]');
    % title(['Translation Error in ' axis(3) ' Axis']);
    % legend;

    % %% Plotting rotational error
    % figure;

    % % X Axis
    % subplot(3, 1, 1);
    % plot(t, rot_err(:, 1), 'DisplayName', 'Rotation Error', 'LineWidth', 1);
    % hold on;
    % fill([t'; flipud(t')], [+3 * sqrt(var_rx); flipud(-3 * sqrt(var_rx))], 'r', ...
    %     'FaceAlpha', 0.2, 'EdgeColor', 'none');
    % plot(t, +3 * sqrt(var_rx), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope upper bound');
    % plot(t, -3 * sqrt(var_rx), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope lower bound');
    % xlabel('Timestep');
    % ylabel('Error [rad]');
    % title(['Rotation Error in ' axis(1) ' Axis']);
    % legend;

    % % Y Axis
    % subplot(3, 1, 2);
    % plot(t, rot_err(:, 2), 'DisplayName', 'Rotation Error', 'LineWidth', 1);
    % hold on;
    % fill([t'; flipud(t')], [+3 * sqrt(var_ry); flipud(-3 * sqrt(var_ry))], 'r', ...
    %     'FaceAlpha', 0.2, 'EdgeColor', 'none');
    % plot(t, +3 * sqrt(var_ry), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope upper bound');
    % plot(t, -3 * sqrt(var_ry), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope lower bound');
    % xlabel('Timestep');
    % ylabel('Error [rad]');
    % title(['Rotation Error in ' axis(2) ' Axis']);
    % legend;

    % % Z Axis
    % subplot(3, 1, 3);
    % plot(t, rot_err(:, 3), 'DisplayName', 'Rotation Error', 'LineWidth', 1);
    % hold on;
    % fill([t'; flipud(t')], [+3 * sqrt(var_rz); flipud(-3 * sqrt(var_rz))], 'r', ...
    %     'FaceAlpha', 0.2, 'EdgeColor', 'none');
    % plot(t, +3 * sqrt(var_rz), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope upper bound');
    % plot(t, -3 * sqrt(var_rz), 'r--', 'LineWidth', 0.5, 'DisplayName', 'Uncertainty envelope lower bound');
    % xlabel('Timestep');
    % ylabel('Error [rad]');
    % title(['Rotation Error in ' axis(3) ' Axis']);
    % legend;

end

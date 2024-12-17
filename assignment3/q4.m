load dataset3.mat


% y_k_j = y_k_j(:, 1215:1714, :);

disp(size(y_k_j));

total_timestamp = size(y_k_j, 2);
disp(total_timestamp);

scatter_colors = cell(1, total_timestamp);

num_visible_landmarks = zeros(1, size(y_k_j, 2));

for k = 1:total_timestamp
    measurements_k = squeeze(y_k_j(:, k,:));
    num_visible_landmarks(1, k) = sum(measurements_k (1 , :) ~= -1);
    
    if num_visible_landmarks(1, k) >= 3
        scatter_colors{k} = 'g';
    else
        scatter_colors{k} = 'r';
    end
end

disp(size(num_visible_landmarks));

    good_observations = scatter(t(num_visible_landmarks >=3), num_visible_landmarks(num_visible_landmarks >=3) , 20, 'g', 'filled'); 
    bad_observations = scatter(t(num_visible_landmarks < 3), num_visible_landmarks(num_visible_landmarks < 3) , 20, 'r', 'filled');
% Create Scatter Plot
figure; hold on;
for t = 1:k
    scatter(t, num_visible_landmarks(t), 20, scatter_colors{t}, 'filled'); % x = t, y = value, color
end
hold off;

% Add Labels
xlabel('Time Step');
ylabel('Number of Visible Landmarks');
title('Scatter Plot of Visible Landmarks v.s.Timestep');

grid on;
ylim([0, max(num_visible_landmarks) + 1]);
saveas(gcf, 'q4_all_timestamps.png'); % 'gcf' gets the current figure handle
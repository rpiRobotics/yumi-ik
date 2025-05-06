Q_deg_in = rad2deg(Q_in');
Q_deg_out = rad2deg(Q_out');

% Combine
Q_all = [Q_deg_in; Q_deg_out];

% Total number of solutions
num_total = size(Q_all, 1);
num_in = size(Q_deg_in, 1);

% Generate LaTeX rows
for i = 1:num_total
    vals = Q_all(i, :);
    if i <= num_in
        % Bold row number and joint values
        joint_strs = arrayfun(@(x) sprintf('\\textbf{%7.2f}', x), vals, 'UniformOutput', false);
        row = sprintf('\\textbf{%2d} & %s \\\\', i, strjoin(joint_strs, ' & '));
    else
        % Regular formatting
        joint_strs = arrayfun(@(x) sprintf('%7.2f', x), vals, 'UniformOutput', false);
        row = sprintf('%2d  & %s \\\\', i, strjoin(joint_strs, ' & '));
    end
    disp(row)
end
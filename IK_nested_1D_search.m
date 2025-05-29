[kin, q_min, q_max] = define_yumi;
SEW = yumi.sew_conv_h4([0;0;1]);

% Pick a fixed EE pose and SEW angle
% q = rand_angle([7 1]);
q = deg2rad([-16.60 -8.10 -93.20 79.10 175.80 37.80 199.80]');
[R, p] = fwdkin(kin, q);
psi = SEW.fwd_kin_q(q, kin);

N = 5000;
q1_list = linspace(-pi, pi, N);

Q_path = NaN([7 16 N]);

for i = 1:N
    Q = yumi.IK_given_q1(R, p, kin, q1_list(i));
    if ~isempty(Q)
        Q_path(:, 1:width(Q), i) = Q;
    end
end
%%



psi_path = NaN([16, N]);

for soln_num = 1:16
for i = 1:N
    q_i = Q_path(:,soln_num, i);
    % q_i = yumi.filter_Q_joint_limits(q_i, q_min, q_max);
    psi_path(soln_num, i) = SEW.fwd_kin_q(q_i, kin);
end
end
%%

h_fig = figure(10);
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
figure_size = 2*[3.5 3.5];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs
% set(h_fig, 'renderer', 'painters')

plot(q1_list, psi_path, 'k.', MarkerSize=2);

% threshold = 0.25;
% hold on;
% for i = 1:size(psi_path, 1)
%     y = psi_path(i, :);
%     dy = abs(diff(y));
% 
%     % Insert NaNs where jump exceeds threshold
%     y_plot = y;
%     y_plot([false, dy > threshold]) = NaN;
% 
%     % Plot line
%     plot(q1_list, y_plot, 'k');
% end
% hold off


yline(psi);
% xline((q(1)));

xlabel("$q_1$", Interpreter="latex")
ylabel("$\psi^{conv}$", Interpreter="latex")
xlim([-pi, pi])
ylim([-pi, pi])

yticks([-pi, -pi/2, 0, pi/2, pi])
yticklabels({'$-\pi$', '$-\frac{\pi}{2}$',  '$0$','$\frac{\pi}{2}$', '$\pi$'})

xticks([-pi, -pi/2, 0, pi/2, pi])
xticklabels({'$-\pi$', '$-\pi/2$',  '$0$','$\pi/2$', '$\pi$'})


fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

load("example_poses\yumi_4_6.mat", "Q_in", "Q_out")

hold on
plot(Q_in(1,:),psi*ones(size(Q_in(1,:))), 'ok', MarkerSize=10)
plot(Q_in(1,:), psi*ones(size(Q_in(1,:))), 'xk', MarkerSize=10)
plot(Q_out(1,:), psi*ones(size(Q_out(1,:))), 'or', MarkerSize=10)
plot(Q_out(1,:), psi*ones(size(Q_out(1,:))), 'xr', MarkerSize=10)
hold off

%%
exportgraphics(h_fig,'output/IK_nested_1D_search.png','Resolution',900)
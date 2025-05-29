[Q, is_LS_vec] = yumi.IK_SEW_mex(R_07, p_0T, SEW, psi, kin, true, 1000)
%%


% Q_filter = unique_q_tol(yumi.filter_Q_joint_limits([q Q], q_min, q_max, mode='remove'), deg2rad(0.1), "infinity")
Q_filter = unique_q_tol([q Q], deg2rad(0.05), "infinity")
%%
% view(2)

h_fig = figure(10);
tiledlayout(1,1,'TileSpacing','none','Padding','compact');
nexttile
figure_size = 2*[3.5 2];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs
set(h_fig, 'renderer', 'painters')

%%
% view(2)
% box on
% 
% ylim([-pi/3, pi/3]);
% xlim([-pi, pi]);
% xlabel("$q_1$", Interpreter="latex");
% ylabel("$q_2$", Interpreter="latex");
% 
% yticks([-pi/3, -pi/6, 0, pi/6, pi/3])
% yticklabels({'$-\pi/3$', '$-\pi/6$',  '$0$','$\pi/6$', '$\pi/3$'})
% 
% xticks([-pi, -pi/2, 0, pi/2, pi])
% xticklabels({'$-\pi$', '$-\pi/2$',  '$0$','$\pi/2$', '$\pi$'})
% 
% 
% fontsize(2*8, 'points')
% xaxisproperties= get(gca, 'XAxis');
% xaxisproperties.TickLabelInterpreter = 'latex';
% yaxisproperties= get(gca, 'YAxis');
% yaxisproperties.TickLabelInterpreter = 'latex';
% 
% hold on
% plot3(Q_filter(1,:), Q_filter(2,:),2*ones(size(Q_filter(1,:))), 'ok')
% plot3(Q_filter(1,:), Q_filter(2,:),2*ones(size(Q_filter(1,:))), 'xk')
% hold off
% 
% 
% h_fig = gcf
% figure_size = 2*[3.5 3.5;];
% set(h_fig, "Units", "inches")
% pos_old = h_fig.OuterPosition;
% if ~all(pos_old(3:4) == figure_size)
% set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
% end
% set(h_fig, "Units", "pixels")
% findfigs

%% LARGER view
view(2)
box on

% Flip around view so we're looking at the bottom i.e. the minimum values
view(0,-90)
set(gca, 'YDir', 'reverse'); 

cb = colorbar('FontSize',12, 'TickLabelInterpreter','latex');
colormap turbo
clim([0 2]);

ylim([-pi, pi]);
xlim([-pi, pi]);
xlabel("$q_1$", Interpreter="latex");
ylabel("$q_2$", Interpreter="latex");

yticks([-pi, -pi/2, 0, pi/2, pi])
yticklabels({'$-\pi$', '$-\frac{\pi}{2}$',  '$0$','$\frac{\pi}{2}$', '$\pi$'})

xticks([-pi, -pi/2, 0, pi/2, pi])
xticklabels({'$-\pi$', '$-\pi/2$',  '$0$','$\pi/2$', '$\pi$'})


fontsize(2*8, 'points')
xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

hold on
% plot3(Q_filter(1,:), Q_filter(2,:),2*ones(size(Q_filter(1,:))), 'ok')
% plot3(Q_filter(1,:), Q_filter(2,:),2*ones(size(Q_filter(1,:))), 'xk')
plot3(Q_in(1,:), Q_in(2,:),-2*ones(size(Q_in(1,:))), 'ok')
plot3(Q_in(1,:), Q_in(2,:),-2*ones(size(Q_in(1,:))), 'xk')
plot3(Q_out(1,:), Q_out(2,:),-2*ones(size(Q_out(1,:))), 'or')
plot3(Q_out(1,:), Q_out(2,:),-2*ones(size(Q_out(1,:))), 'xr')
hold off


h_fig = gcf;
figure_size = 2*[3.5 3.5;];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs
%%
exportgraphics(h_fig,'output/plot_2D_search.png','Resolution',900)

%%
exportgraphics(h_fig,'output/plot_2D_search_2.png','Resolution',900)
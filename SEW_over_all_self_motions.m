%% Proof of instantaneous augmentation singularities

[kin, q_min, q_max] = define_yumi;

% Pick a fixed EE pose
% q = rand_angle([7 1]);
q = deg2rad([0 -31.12 61.30 -65.33 -132.67 -20.55 0]');
[R, p] = fwdkin(kin, q);


% IK over all choices of q_1
N = 1000;
q1_list = linspace(-pi, pi, N);
% q1_list = linspace(-0.75, 0.5, N);
% q1_list = linspace(-0.572322, -0.571071, N);
Q_path = NaN([7 16 N]);

for i = 1:N

    Q = yumi.IK_given_q1(R, p, kin, q1_list(i));
    if ~isempty(Q)
        Q_path(:, 1:width(Q), i) = Q;
    end

end
%%
Q_path_filter = yumi.filter_Q_joint_limits(Q_path, q_min, q_max);
q2_list = squeeze(Q_path_filter(2,:,:));
plot(q1_list, q2_list', '.')
xline(q_min(1));
xline(q_max(1));
yline(q_min(2));
yline(q_max(2));
xline(0)
%%
% Calculate SEW angle over ALL paths

SEW = yumi.sew_conv_h4([0;0;1]);

psi_path = NaN([16, N]);
det_J_path = NaN([16, N]);
det_JJT_path = NaN([16, N]);
Q_7_filter = NaN([7 16 N]);

for soln_num = 1:16
for i = 1:N
    q_i = Q_path(:,soln_num, i);
    q_i = yumi.filter_Q_joint_limits(q_i, q_min, q_max);
    Q_7_filter(:,soln_num, i) = q_i;
    psi_path(soln_num, i) = SEW.fwd_kin_q(q_i, kin);

    [J_A_i, J_6x7] = SEW.J_aug(q_i, kin);
    det_J_path(soln_num, i) = det(J_A_i);
    det_JJT_path(soln_num, i) = det(J_6x7* J_6x7');
end
end
%%
plot(q1_list, psi_path, '.'); hold on;

yline(0);
xlim([-pi, pi])
ylim([-pi, pi])
xline(q_min(1));
xline(q_max(1));
xlabel("q_1")
ylabel("\psi")
hold off
%%
plot(q1_list, psi_path, '.'); hold on;
set(gca,'ColorOrderIndex',1)
plot(q1_list, det_J_path/max(det_J_path, [], 'all'), 'x');
plot(q1_list, det_JJT_path/max(det_JJT_path, [], 'all'), 'b.');
plot(q1_list, squeeze(Q_7_filter(2,:,:)), 'r.');
plot(q1_list, squeeze(Q_7_filter(6,:,:)), 'g.');
yline(0);
% xlim([-pi, pi])
ylim([-pi, pi])
% xline(q_min(1));
% xline(q_max(1));
xlabel("q_1")
ylabel("\psi")
hold off

%% Look at a single pose

% plot( det_J_path'/max(det_J_path, [], 'all'), 'x');
% vpa(Q_7_filter(:,2,665)')
q = [-0.57149050450450444849082032305887, -0.54313521459076008479627262204303, 1.0699264551465166306343235191889, -1.1402111036067774652025264003896, -2.3155267940093628986630847066408, -0.35866861896458357428230101504596, 3.0603979606700311144606985180872]';
q_deg = round(rad2deg(q),2)
q = deg2rad(q_deg);

q(1)=0;
q(7)=0;

[J_A, J, J_psi] = SEW.J_aug(q, kin)
svd(J)
% [U,S,V] = svd(J)
svd(J_A)
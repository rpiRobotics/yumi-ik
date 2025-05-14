[kin, q_min, q_max] = define_yumi;
SEW = yumi.sew_conv_h4([0;0;1]);

% Pick a fixed EE pose and SEW angle
% q = rand_angle([7 1]);
q = deg2rad([-16.60 -8.10 -93.20 79.10 175.80 37.80 199.80]');
[R, p] = fwdkin(kin, q);
psi = SEW.fwd_kin_q(q, kin);

N = 10000;
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

plot((q1_list), psi_path, '.');
yline(psi);
xline((q(1)));

xlabel("q_1")
ylabel("\psi^{conv}")
xlim([-pi, pi])
ylim([-pi, pi])
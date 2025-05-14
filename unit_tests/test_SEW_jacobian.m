%% Compare Jacobian to finite difference approximation

kin = define_yumi;

N = 1e3;
err_list = NaN([1 1e3]);

for i = 1:N
e_r = rand_normal_vec;
% e_r = [0;0;1];
q = rand_angle([7 1]);

SEW = yumi.sew_conv_h4(e_r);

delta = 1e-6;
direction = rand_angle([7 1]); direction = direction / norm(direction);
delta_vec = delta*direction;

diff_numeric = (SEW.fwd_kin_q(q+delta_vec/2, kin) - SEW.fwd_kin_q(q-delta_vec/2, kin)) / delta;
diff_analytical = SEW.full_J_psi(q, kin) * direction;
err_list(i) = diff_numeric - diff_analytical;
end

semilogy(sort(abs(err_list)))
kin  = define_yumi;
% SEW = yumi.sew_conv_h4([0;0;1]);
SEW = yumi.sew_conv_h4(rand_normal_vec);
%% Baseline

q = rand_angle([7 1]);
J_A = SEW.J_aug(q, kin);
det(J_A)
%% q_2 singularity
% Joints 1 and 3 become collinear

q = rand_angle([7 1]);
q(2) = 0;
J_A = SEW.J_aug(q, kin);
det(J_A)

%% q_6 singularity
% Joints 5 and 7 become collinear

q = rand_angle([7 1]);
q(6) = 0;
J_A = SEW.J_aug(q, kin);
det(J_A)

%% q4 (not a singularity)

q = rand_angle([7 1]);
q(4) = -pi/2;
J_A = SEW.J_aug(q, kin);
det(J_A)


%% Case B1A from Asgari paper

q = rand_angle([7 1]);
q(4) = -pi/2;
q(6) = 0;
J_A = SEW.J_aug(q, kin);
det(J_A)



%% Investigate what self-motion looks like using 6x7 matrix
J = robotjacobian(kin, q)
null(J)

J_A * null(J) % How does each self-motion velocity affect SEW angle?
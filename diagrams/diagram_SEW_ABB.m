ex = [1; 0; 0];
ey = [0; 1; 0];
ez = [0; 0; 1];
zv = [0; 0; 0];


kin = define_yumi();
kin.P = kin.P / 100;
R_7T = round(rot(ey, pi/2));


e_r = ez;

CYL_HALF_LENGTH = 0.2;
CYL_RADIUS = 0.1;



% q = deg2rad(45)*ones([7 1]);
% q = zeros([7 1]);
q = deg2rad([0 -60 0 60 0 0 0])';

R_01 = rot(kin.H(:,1), q(1));
R_12 = rot(kin.H(:,2), q(2));
R_23 = rot(kin.H(:,3), q(3));
R_34 = rot(kin.H(:,4), q(4));
R_45 = rot(kin.H(:,5), q(5));
R_56 = rot(kin.H(:,6), q(6));
R_67 = rot(kin.H(:,7), q(7));

R_02 = R_01 * R_12;
R_03 = R_02 * R_23;
R_04 = R_03 * R_34;
R_05 = R_04 * R_45;
R_06 = R_05 * R_56;
R_07 = R_06 * R_67;
R_0T = R_07 * R_7T;

p_01 = kin.P(:,1);
p_02 = p_01 + R_01 * kin.P(:,2);
p_03 = p_02 + R_02 * kin.P(:,3);
p_04 = p_03 + R_03 * kin.P(:,4);
p_05 = p_04 + R_04 * kin.P(:,5);
p_06 = p_05 + R_05 * kin.P(:,6);
p_07 = p_06 + R_06 * kin.P(:,7);
p_0T = p_07 + R_07 * kin.P(:,8);


diagrams.setup(); hold on


diagrams.dot(zv, color=diagrams.colors.red);
diagrams.dot(p_01, color=diagrams.colors.red);
diagrams.dot(p_02, color=diagrams.colors.red);
diagrams.dot(p_03, color=diagrams.colors.red);
diagrams.dot(p_04, color=diagrams.colors.red);
diagrams.dot(p_05, color=diagrams.colors.red);
diagrams.dot(p_06, color=diagrams.colors.red);
diagrams.dot(p_07, color=diagrams.colors.red);
diagrams.dot(p_0T, color=diagrams.colors.red);

diagrams.line(ez, p_01, LineStyle=':');

% c1 = diagrams.cylinder(p_01, kin.H(:,1), CYL_HALF_LENGTH, CYL_RADIUS);
c1 = diagrams.cylinder(zv+CYL_HALF_LENGTH*ez, kin.H(:,1), CYL_HALF_LENGTH, CYL_RADIUS);
c2 = diagrams.cylinder(p_02, R_01*kin.H(:,2), CYL_HALF_LENGTH, CYL_RADIUS);
c3 = diagrams.cylinder(p_03, R_02*kin.H(:,3), CYL_HALF_LENGTH, CYL_RADIUS);
c4 = diagrams.cylinder(p_04, R_03*kin.H(:,4), CYL_HALF_LENGTH, CYL_RADIUS);
c5 = diagrams.cylinder(p_05, R_04*kin.H(:,5), CYL_HALF_LENGTH, CYL_RADIUS);
c6 = diagrams.cylinder(p_06, R_05*kin.H(:,6), CYL_HALF_LENGTH, CYL_RADIUS);
c7 = diagrams.cylinder(p_07, R_06*kin.H(:,7), CYL_HALF_LENGTH, CYL_RADIUS);

diagrams.cylinder_line(c1, c2, LineWidth=8, color=diagrams.colors.blue);
diagrams.cylinder_line(c2, c3, LineWidth=8, color=diagrams.colors.blue);
diagrams.cylinder_line(c3, c4, LineWidth=8, color=diagrams.colors.blue);
diagrams.cylinder_line(c4, c5, LineWidth=8, color=diagrams.colors.blue);
diagrams.cylinder_line(c5, c6, LineWidth=8, color=diagrams.colors.blue);
diagrams.cylinder_line(c6, c7, LineWidth=8, color=diagrams.colors.blue);
diagrams.line(p_07+R_06*kin.H(:,7)*CYL_HALF_LENGTH, p_0T, LineWidth=8, color=diagrams.colors.blue);


diagrams.arrow(p_04 + R_03*kin.H(:,4)*CYL_HALF_LENGTH,   p_04 + R_03*kin.H(:,4)*(CYL_HALF_LENGTH+1));
diagrams.text( p_04 + R_03*kin.H(:,4)*(CYL_HALF_LENGTH+0.75), "$\vec h_4$", align='v', margin=6);

diagrams.arrow(p_01, p_01+e_r);
diagrams.text(p_01 + e_r, "$\vec e_r$", align='v', margin=0);

diagrams.arrow(p_01, p_07);
diagrams.text(p_01 + (-p_01 + p_07)*0.4, "$\vec p_{SW}$", align='>');

diagrams.text(0.05*ex, "$\mathcal O_0$", align='^', margin=8);
diagrams.text(p_01, "$\mathcal O_1 = \mathcal O_S$", align='>');
diagrams.text(p_07, "$\mathcal O_7 = \mathcal O_W$", align='v', margin=14);


diagrams.arrow(zv, ex);
diagrams.arrow(zv, ey);
diagrams.arrow(zv, ez);
diagrams.text(ex, "$\vec e_{x,0}$", align='v', margin=10);
diagrams.text(ey, "$\vec e_{y,0}$", align='<v', margin=2);
diagrams.text(ez, "$\vec e_{z,0}$", align='>^');

diagrams.arrow(p_0T, p_0T + R_0T * ex);
diagrams.arrow(p_0T, p_0T + R_0T * ey);
diagrams.arrow(p_0T, p_0T + R_0T * ez);
diagrams.text(p_0T + R_0T * ex, "$\vec e_{x,T}$", align='>v');
diagrams.text(p_0T + R_0T * ey, "$\vec e_{y,T}$", align='^>', margin=-2);
diagrams.text(p_0T + R_0T * ez*0.75, "$\vec e_{z,T}$", align='v');
diagrams.text(p_0T, "$\mathcal O_T$", align='>v', margin=0);




view(160,30)
diagrams.redraw(); hold off

%%
diagrams.save();
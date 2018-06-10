function x_y_z_d = intersection_pt(pt,v1,v2)

%% Author : Mandeep Singh
% email : mandeep14145@iiitd.ac.in

%%
syms t

a = v1 - v2;
r = v2 + t*a;

normal_vector = r - pt;

eq = dot(normal_vector,a) == 0;

t_ = solve(eq,t);

r = v2 + t_*a;

% d = sqrt((r(1) - pt(1))^2 + (r(2) - pt(2))^2 + (r(3) - pt(3))^2);
x = double(r(1)); y = double(r(2)); z = double(r(3));
x_y_z_d = [x y z];

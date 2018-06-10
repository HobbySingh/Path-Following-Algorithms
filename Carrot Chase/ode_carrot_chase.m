function [dydt] = ode_carrot_chase(t,y,w1_x,w1_y,w2_x,w2_y)

%% Author - Mandeep Singh
% email - mandeep14145@iiitd.ac.in

%% Initializing Params
dydt = zeros(3,1);

v = 10;
delta = 10;
k = 60;

ugv_x = y(1);
ugv_y = y(2);
si = y(3);

%% XY Controller 

% Computing the position error
% Distance of point to line (UGV Position - Desired path)

R_u = sqrt((w1_x - ugv_x)^2 + (w1_y - ugv_y)^2);
theta = atan2((w2_y - w1_y),(w2_x - w1_x));
theta_u = atan2(ugv_y - w1_y,ugv_x - w1_x);
beta = theta - theta_u;
R = sqrt(R_u^2 - (R_u*sin(beta))^2);
x_target = w1_x + (R+delta)*cos(theta); 
y_target = w1_y + (R+delta)*sin(theta);
% pause(0.01)
si_d = atan2(y_target - ugv_y, x_target - ugv_x);
u = k*(si_d - si);
%% Non-Holonomic COnstraints
% Constraining the control input
% Rmin = 125;
% if(abs(u) > (v^2)/Rmin)
%     if (u > 0)
%         u = (v^2)/Rmin;
%     else
%         u = -(v^2)/Rmin;
%     end
% end

% Finally heading angle
si_dot = u;

%% STATE EQUATIONS
dydt(1) = v*cos(si); % y(1) -> uav_x
dydt(2) = v*sin(si); % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
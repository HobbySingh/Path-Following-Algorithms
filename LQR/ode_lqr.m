function [dydt] = ode_lqr(t,y,w1_x,w1_y,w2_x,w2_y,c)

%% Author - Mandeep Singh
%  Email - mandeep14145@iiitd.ac.in

%% Initializating Params
dydt = zeros(4,1);

v = 10;

db = 4;
q2 = 1;
Rmin = 75;

%% XY Controller 

% Computing the position error
% Distance of point to line (UGV Position - Desired path)

pt = [y(1) y(2) 0]; % UGV Positon vector
a1 = [w1_x w1_y 0]; % Waypoint 1 Vector
a2 = [w2_x w2_y 0]; % Waypoint 2 Vector

tmp = (y(1) - w1_x)*(w2_y - w1_y) - (y(2) - w1_y)*(w2_x - w1_x);
% To check whether the point is left or right of the desired path
if(tmp < 0)
    d = point_to_line(pt,a1,a2);
else
    d = -point_to_line(pt,a1,a2);
end    
% d = y(4) 

% LQR Formualtion
q1 = sqrt(abs(db/(db - d)));
% k = 0.0001;
% q1 = sqrt(exp(k*(abs(d))));
si = y(3);
si_p = atan2((w2_y - w1_y),(w2_x - w1_x)); % si desired
vd = v*sin(si - si_p); % d_dot
u = -(q1*d + sqrt(2*q1 + q2^2)*vd);

% Constraining the control input
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
dydt(4) = vd; % y(4) -> d
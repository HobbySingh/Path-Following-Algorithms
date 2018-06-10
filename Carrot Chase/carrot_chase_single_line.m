clc; clear all;
close all;

%% Initialization of params
 waypoints = [[300, 300, 0]; [0, 0, 0]; [150, 600, 0];[-300,600,0]; [0, 0, 0]; [300,0,0]; [0, 0, 0];];

wp_1 = waypoints(1,:);
w1_x = wp_1(1); w1_y = wp_1(2);

wp_2 = waypoints(2,:);
w2_x = wp_2(1); w2_y = wp_2(2);

lw = 1;
tspan = 0:0.1:25;

%% Straight Line Initial Condition
curr_x = 200;
curr_y = 300;
curr_si= 1.57;
delta = 25;

%% Plotting Reference Trajectory
fprintf("Plotting trajectories\n");
% arr_x = (300:-0.5:150);
% arr_y = (300:1:600);
arr_x = (0:1:300);  
arr_y = (0:1:300);

plot(arr_x(1,:),arr_y(1,:),'--k'); 

%% Path following begins
y0 = [curr_x curr_y curr_si] ;
[t,y] = ode45(@(t,y) ode_carrot_chase(t,y,w1_x,w1_y,w2_x,w2_y), tspan, y0);

hold on 
grid on
for i = 1:length(y(:,1))-1
    plot(y(i:i+1,1),y(i:i+1,2),'-m','LineWidth',lw);
    pause(0.01)
end

% title('LQR based Path Following')
% legend ('Desired Path','Path (Adaptive Optimal Guidance Law)');
xlabel('X(m)') % x-axis label
ylabel('Y(m)') % y-axis label

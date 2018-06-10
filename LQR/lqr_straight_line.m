%% 
% Coded by : Mandeep Singh
% Institue : IIITD 

%% Clear All 
clc; clear all;
close all;

%% Initialization of params
 waypoints = [[0, 0, 0]; [300, 300, 0]; [150, 600, 0];[-300,600,0]; [0, 0, 0]; [300,0,0]; [0, 0, 0];];
counter = 0;
w_spd_ratio = 0.2;

wp_1 = waypoints(1,:);
w1_x = wp_1(1); w1_y = wp_1(2); w1_z = wp_1(3);

wp_2 = waypoints(2,:);
w2_x = wp_2(1); w2_y = wp_2(2); w2_z = wp_2(3);

lw = 1;
tspan = 0:0.05:0.2;

% Straight Line Initial Condition
curr_x = 50;
curr_y = 0;
curr_si =0;
curr_d = 0;
curr_z = 0;
curr_si_z = 0;
curr_d_z = 0;    
dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
delta = 4;

%% Plotting Reference Trajectory
wp = 1;
fprintf("Plotting trajectories\n");

%% Path following begins
wp = 2;
while ( dist_wp > delta && wp <= 5)
    y0 = [curr_x curr_y curr_si curr_d] ;
    [t,y] = ode45(@(t,y) ode_lqr(t,y,w1_x,w1_y,w2_x,w2_y,0), tspan, y0);
    curr_x = y(end,1);
    curr_y = y(end,2);
    curr_si = y(end,3);
    curr_d = y(end,4);
    
    dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);

    if(dist_wp < delta)
        fprintf("waypoint reached: %d \n", wp);
        wp_1 = waypoints(wp,:);
        w1_x = wp_1(1); w1_y = wp_1(2);

        wp_2 = waypoints(wp+1,:);
        w2_x = wp_2(1); w2_y = wp_2(2);
        
        pt = [curr_x,curr_y, 0];
        v1 = [w1_x w1_y 0];
        v2 = [w2_x w2_y 0];
        d = point_to_line(pt,v1,v2);

        dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 );

        wp = wp + 1;
        curr_si = y(end,3);
        curr_d = d;
    end
%     hold on
    plot(y(:,1),y(:,2),'-m','LineWidth',lw);
    counter = counter + 1;
    if(wp <= 5)
        p1 = [waypoints(wp-1,1),waypoints(wp,1)];
        p2 = [waypoints(wp-1,2),waypoints(wp,2)];
        p3 = [waypoints(wp-1,3),waypoints(wp,3)];
        plot(p1,p2,'--k','LineWidth',1);
    end
    pause(0.01);
    hold on
    grid on
end
hold on
title('LQR based Path Following')
legend ('Desired Path','Path (Adaptive Optimal Guidance Law)');
xlabel('X(m)') % x-axis label
ylabel('Y(m)') % y-axis label
xlim([-400 400])
ylim([-100 750])

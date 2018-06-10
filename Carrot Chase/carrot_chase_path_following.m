%% Author - Mandeep Singh
% email - mandeep14145@iiitd.ac.in

%% Clear All
clc; clear all;
close all;

%% Initialization of params
 waypoints = [[0, 0, 0]; [300, 300, 0]; [150, 600, 0];[-300,600,0]; [0, 0, 0]; [300,0,0]; [0, 0, 0];];

wp_1 = waypoints(1,:);
w1_x = wp_1(1); w1_y = wp_1(2);

wp_2 = waypoints(2,:);
w2_x = wp_2(1); w2_y = wp_2(2);

lw = 1;
tspan = 0:0.01:1;

% Straight Line Initial Condition
curr_x = 50;
curr_y = 0;
curr_si =0;
dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2);
delta = 5;

%% Plotting Reference Trajectory
fprintf("Plotting trajectories\n");

%% Path following begins
wp = 2;
while ( dist_wp > delta && wp <= 5)
    y0 = [curr_x curr_y curr_si] ;
    [t,y] = ode45(@(t,y) ode_carrot_chase(t,y,w1_x,w1_y,w2_x,w2_y), tspan, y0);
    curr_x = y(end,1);
    curr_y = y(end,2);
    curr_si = y(end,3);
    
    dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2);

    if(dist_wp < delta)
        fprintf("waypoint reached: %d \n", wp);
        
        wp_1 = waypoints(wp,:);
        w1_x = wp_1(1); w1_y = wp_1(2);

        wp_2 = waypoints(wp+1,:);
        w2_x = wp_2(1); w2_y = wp_2(2);
        
        dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 );

        wp = wp + 1;

    end
%     hold on
    plot(y(:,1),y(:,2),'-m','LineWidth',lw);
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
% hold on
title('Carrot Chase based Path Following')
legend ('Desired Path','Path (delta = 10,k = 60)');
xlabel('X(m)') % x-axis label
ylabel('Y(m)') % y-axis label
xlim([-400 400])
ylim([-100 750])

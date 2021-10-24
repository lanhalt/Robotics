%% EK505 Homework 2, Problem 1
%Laura Anhalt, U85350496

%% Create sphere world topology
% tiledlayout(2,2);
% ax1 = nexttile;
% sphere(ax1);
% axis equal
% title('20-by-20 faces (Default)')
% [X,Y] = meshgrid(-50:2:50,-50:2:50);
% Z = X.^2+Y.^2
% surf(X,Y,Z);
% figure
% contour(X,Y,Z,30);
% 
% hold on

% % Create arrays
% x = linspace(1,20,20);
% y = linspace(1,20,20);
% plot(x,y)
% d_goal = [18, 16];
% d_start = [1, 1];

%% Global Variables
clear 
MapR = 10;   %radius of boundary
theta = 0:pi/50:2*pi;  %vector of steps to make a full circle in radians
q_start = [5,0];  %cartesian coordinate [x,y];
q_goal = [-6,-6];  %cartesian coordinate [x,y];
zeta = 2;  %zeta for Uatt, can be changed as needed
Eta = 5;  %eta for Urep, can be changed as needed
Q_s = 2;  %Q* for Urep, can be changed as needed

%% create obstacles 
ob1 = circle(3,5,1);
ob2 = circle(-5,1,2);
ob3 = circle(-1,-8,1);
%also a plot for the world boundary
world = circle(0,0,MapR);
%plot
grid on
hold on
plot(world(:,1),world(:,2), 'b')
plot(ob1(:,1),ob1(:,2))
plot(ob2(:,1),ob2(:,2))
plot(ob3(:,1),ob3(:,2))
plot(q_start(1), q_start(2), 'x')
plot(q_goal(1), q_goal(2),'x')
title('Map for Robot')
hold off

% Create arrays
[qx,qy] = meshgrid(-10:.5:10,-10:.5:10);  %create grid of space for x and y
% qx = linspace(-10,10,41);
% qy = linspace(-10,10,41);
% q= [qx' qy'];  %space that spans the boundary

%% Caluclate Uatt (Quadratic)
%UA = zeros(41,1);  %preallocate matrix
d = sqrt((qx-q_goal(1)).^2 + (qy-q_goal(2)).^2);  %calc disatance from positions to goal
UA = .5.*zeta.*d.^2;   %calc Uattraction at all those positions
figure
contour(qx,qy,UA,30)   %plot the potential contours
title('Attractive Potential Function Contours')

figure 
surf(qx,qy,UA)
title('Attractive Potential Function Surface')


% for i = 1:10
%     q(:,2) = i;
%     UA(:,i) = quad_attraction(q_goal,q,Z);   %each column holds the scalar values of UA for the positions around a circle of radius 1->10. column1=radius1
% end 

% 
% %% Calculate Gradient Uatt (Quadratic)
% for i = 1:10
%     q(:,2) = i;  %re-allocate the positon to start at radius 1
%     [qx,qy] = pol2cart(q(:,1), q(:,2));  %convert from polar to cartesian coordinates
%     qpos = [qx qy];
%     delUA(:,i*2-1:i*2) = Z.*(qpos-q_goal);
% end 

%% Function to make circle points for plotting
% takes in origin and radius and spits out x and y coordinates to needed to
% plot a circle
    function coord = circle(x,y,r)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    coord = [xunit' yunit'];
    end 
%% Function to find the shortest distance to the obsatcle and the point on that obstacle
%inputs are the obstacle x-y coordinates (martrix nx2) and the posisiton(vector) you're at
%outputs are a x-y vector for c and the scalar distance value
    function [c, d] = ob_dist(ob_coord, pos)
    A = zeros(length(ob_coord),1);
    for i=1:length(ob_coord)
        A(i,:) = norm(ob_coord(i,:)-pos);
    end
    d = min(A);
    k = find(A==d);  %find index in A that gave you the min distance
    c = ob_coord(k,:);  %use that index to pull the corresponding coordinates from the ob_coord matrix
    end 

%% Function to calculate gradientD
%inputs are point location you're at (q) and the point on the obstacle that derives the shorstest distance (c). Both are vector of x-y coordinates
%output is the gradient of d for that obstacle, vector
function del_d = gradientD(pos, c)
del_d = (pos-c)/norm(c-q);
end 

%% Function to calculate Repulsive Potential
function UR = rep_func(Q_s, Eta, d)
if d <= Q_s
    UR = .5.*Eta.*(1./d - 1./Q_s).^2;
else 
    UR = 0;
end
end

%% Function to calculate Gradient Repulsive Potential Function
function del_UR = gradient_rep(Q_s, Eta, del_d, d)
if d <= Q_s
    del_UR = Eta.*(1./Q_s - 1./d).*(1./(d.^2)).*del_d;
else 
    del_UR = 0;
end 
end 

% %% Function for Calculating U attraction 
% %starting with just quadratic attractions first 
% function UA = quad_attraction(goal,position,Z)
% [qx,qy] = pol2cart(position(:,1), position(:,2));  %convert from polar to cartesian coordinates
% d = sqrt((qx-goal(1)).^2 + (qy-goal(2)).^2);  %calc disatance from position to goal
% UA = .5.*Z.*d.^2;   %calc Uattraction at all those positions, this will give out a column vector
% end 
% 
% %% Function for Calcualting Gradient U attraction
% % function delUA = del_quad_att(goal,position,Z)
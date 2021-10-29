%% EK505 Homework 2, Problem 1
%Laura Anhalt, U85350496


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
%plot everything to start
figure
grid on
hold on
plot(world(:,1),world(:,2), 'b')
plot(ob1(:,1),ob1(:,2))
plot(ob2(:,1),ob2(:,2))
plot(ob3(:,1),ob3(:,2))
plot(q_start(1), q_start(2), 'gx')
plot(q_goal(1), q_goal(2),'rx')
title('Map for Robot')
legend('World','Ob1','Ob2','Ob3','Start','Goal')
hold off

% Create arrays
[qx,qy] = meshgrid(-10:.5:10,-10:.5:10);  %create grid of space for x and y
% qx = linspace(-10,10,41);
% qy = linspace(-10,10,41);
% q= [qx' qy'];  %space that spans the boundary

%% Caluclate Uatt (Quadratic)
%UA = zeros(41,1);  %preallocate matrix
d = sqrt((qx-q_goal(:,1)).^2 + (qy-q_goal(:,2)).^2);  %calc disatance from positions to goal
UA = .5.*zeta.*d.^2;   %calc Uattraction at all those positions
figure
contour(qx,qy,UA,30)   %plot the potential contours
title('Attractive Potential Function Contours')
figure 
surf(qx,qy,UA)
title('Attractive Potential Function Surface')

%% Calculate Gradient Uatt (Quadratic)
delUAx = zeta.*(qx-q_goal(1));
delUAy = zeta.*(qy-q_goal(2));
%plot of gradient attractive potential
figure
quiver(delUAx,delUAy)
title("Gradient Attractive Potential (Quadratic)")

%% dont think these lines are needed anymore since I got the attractive potential to work above
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

%% Calculate Repulsive Potential Function
% for obstacle 1
qx_vec = linspace(-10,10,41)';
qy_vec = linspace(-10,10,41)';
q_mat = [qx_vec qy_vec];
%calc min dist to ob1 and coordinate on ob1
c1 = zeros(length(ob1), 2);
d1 = zeros(length(ob1),1);
d_qob1 = zeros(length(qx_vec),1);
for i=1:length(qx_vec)
    d_qob1 = sqrt((qx_vec(i)-ob1(:,1)).^2 + (qy_vec(i)-ob1(:,2)).^2);
    d1(i) = min(d_qob1);
    k = find(d_qob1==d1(i));
    c1(i,:) = ob1(k,:);
end 
%calc repulsive potential
UR_1 = zeros(length(qx_vec),1);  %preallocate the repulsive function
for i=1:length(qx_vec)
    if d1(i) <= Q_s
        UR_1(i) = .5.*Eta.*((1./d1(i))-(1./Q_s)).^2;
    else 
        UR_1(i) = 0;
    end
end
%?How do I get UR into a format that I can make a contour plot out of it??

%calc gradient of distance
del_d1x = zeros(length(qx_vec),1);
del_d1y = zeros(length(qy_vec),1);
% for i=1:length(qx_vec)
%     del_d1x(i) = (qx_vec(i)-c1(i,1))./sqrt((qx_vec(i)-c1(i,1)).^2 + (qy_vec(i)-c1(i,2)).^2);
% end
del_d1 = (q_mat-c1)./(sqrt((qx_vec-c1(:,1)).^2 + (qy_vec-c1(:,2)).^2));

delUR_1 = zeros(length(qx_vec),2);
for i = 1:length(qx_vec)
    if d1(i) <= Q_s
        delUR_1(i,:) = Eta.*((1./Q_s) - (1./d1(i))).*(1./(d(i).^2)).*del_d1(i,:);
    else 
        delUR_1(i,:) = 0;
    end
end
%?? How do I reshape this to a meshgrid in order to plot it??

%% Calcualte Potential Function
%UA and UR are NOT the same shape....so I can't combine them yet until I
%figure that out

%% Scrap code to delete later
% d1 = min(d_qob1);  % min distance to obstacle 1
% k = find(d_qob1==d1);
% c1 = ob1(k,:);  %coordinate on ob1 for min dist


%[c1, d1] = ob_dist(ob1, q_mat);  %this creates an answer, but not the right size?
% for i = 1:length(ob1)
%     for j = 1:length(ob1)
%         [c1(i,j), d(i)] = ob_dist(ob1, q_mat(i,:));
%     end
% end 
%     A = zeros(length(ob1),1);
%     for i=1:length(ob1)
%         A(i) = norm(ob1(i,:)-[qx_vec(i) qy_vec(i)]);
%     end
%     min_dist = min(A);
%     k = find(A==min_dist);  %find index in A that gave you the min distance
%     c = ob1(k,:);


%% Function to make circle points for plotting
% takes in origin and radius and spits out x and y coordinates to needed to
% plot a circle
    function coord = circle(x,y,r)
    s=20;  %increment 
    th = 0:pi/s:2*pi-pi/s;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    coord = [xunit' yunit'];
    end 
%% Function to find the shortest distance to the obsatcle and the point on that obstacle
%inputs are the obstacle x-y coordinates (martrix nx2) and the posisiton(vector 1x2) you're at
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
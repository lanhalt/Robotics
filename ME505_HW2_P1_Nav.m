%% EK505 Homework 2, Problem 1 Part 2
%Laura Anhalt, U85350496
% references are from http://www.cs.bilkent.edu.tr/~culha/cs548/hw2/
global world,
global myrobot,  %starting position of robot
global goal,     %end goal position
global K,        % K for navigation
global epsilon;  %used for ode45 solver, built-in function

% inc=.5; %increment for all the meshgrids and vectors etc
% q_start = [2,-8];  %cartesian coordinate [x,y];
% q_goal = [-6,-6];  %cartesian coordinate [x,y];
% [qx,qy] = meshgrid(-10:inc:10,-10:inc:10);
% qx_vec = transpose(-10:inc:10);
% qy_vec = transpose(-10:inc:10);

world = [ 0 0 10; 3 5 1; -5 1 2; -1 -8 1];
epsilon = 1e-5;
myrobot = [2,-8];
goal = [-6,-6];
K = 3;
%pt = [1 1];

noOfObs = size(world,1);  %number of obstacles

% plot the world and obstacles
for i = 1:noOfObs
[s,r] = sprintf('(x - %f)^2 + (y - %f)^2 - (%f)^2',world(i,1)...
    ,world(i,2),world(i,3));
ezplot(s,[-15 15 -15 15]);
hold on
end
title('Robot Trajectory, Starting Pt (2,-8)');xlabel(''); ylabel('');
plot(myrobot(1),myrobot(2),'go');
hold on
plot(goal(1),goal(2),'r*');
hold on

%% call navigation function with ode
tsp = [0 1e25];

options = odeset('Event',@navEventFunc);
tic
[T,Y] = ode45(@navGrad,tsp,myrobot,options);
toc
%% plot results of robot trajectory on exisiting plot
plot(Y(:,1),Y(:,2),'g','LineWidth',2);
hold off
%% plot Navigation Function Potential
% field resolution
fres = 0.10;
% get the radius of the bounding sphere
worldSize = world(1,3); 

xPs = -worldSize:fres:worldSize ;
yPs = -worldSize:fres:worldSize ;
zPs =  zeros(size(yPs,2),size(xPs,2));

for i=1:size(xPs,2)
    for j=1:size(yPs,2)
       zPs(j,i) = navPotField([xPs(i) yPs(j)]);                 
    end
end
figure(2);
hnd = surf(xPs,yPs,zPs,zPs);
set(hnd,'LineStyle','none');
title('Total Navigation Function Surface, K=3')


%% Functions
function [value,isterminal,direction] = navEventFunc(t,robo)
global epsilon,
global goal;
% gr = navGrad(t,robo)<[epsilon; epsilon];
value = [(norm(navGrad(t,robo))>epsilon)*1  findSqDistance(robo,goal)<epsilon ] ;  
isterminal = [1 1];   % Stop the integration
direction = [0 0];   % any direction
end
function gradnav = navGrad(tspan, robo)
% extern global variables
global goal, % position of the goal
global K; % K for navigation
% current robot location
rbx = robo(1);
rby = robo(2);
% goal location
gx = goal(1);
gy = goal(2);

% find the distance to the goal;
dgoal = findSqDistance(robo,goal);

% find distance to obstacles
dobs = findObsDistance(robo);

% find derivative of obstacle distance
dobsdot = findDerDobs(robo);
 
%% find total gradient

% gradiant=(2*(q-qgoal)*(distance2goal^(2*kapa)+distance2obstacle)^(1/kapa)-...
%           distance2goal^2*(1/kapa)*(distance2goal^(2*kapa) + distance2obstacle)^(1/kapa -1)*(2*kapa*distance2goal^(2*kapa-2)*(q-qgoal)+ObstacleDistanceGradiant(q)))...
%           /(distance2obstacle^(2*kapa)+distance2obstacle)^(2/kapa);
      
xTot = ( 2 * (rbx-gx) * (dgoal^(2*K) + dobs)^(1/K) ...
        - dgoal^2 * (1/K) * (dgoal^(2*K) + dobs)^(1/(K - 1) * ( 2 * K * dgoal^(2*K-2) * (rbx-gx) + dobsdot(1) ) ) ...
        / (dgoal^(2*K) + dobs)^(2/K);
      
yTot = ( 2 * (rby-gy) * (dgoal^(2*K) + dobs)^(1/K) ...
        - dgoal^2 * (1/K) * (dgoal^(2*K) + dobs)^(1/K - 1) * ( 2 * K * dgoal^(2*K-2) * (rby-gy) + dobsdot(2) ) ) ...
        / (dgoal^(2*K) + dobs)^(2/K);

%% return gradnav
gradnav = [-xTot;-yTot];
end
function potent = navPotField(loc)

global goal,
global K;

% distance to goal
dgoal = findSqDistance(loc,goal);
% distance to obstacles
dobs = findObsDistance(loc);

if checkBoundary(loc)
    potent = 1;
else
    if checkObstacle(loc)
        potent = dgoal^2 / ( (dgoal)^(2*K) + dobs )^(1/K);
    else
        potent = 1;
    end
end


end
function distance = findSqDistance(pt1,pt2)

distance = sqrt((pt1(1)-pt2(1))^2+(pt1(2)-pt2(2))^2);

end
function dist = findObsDistance(robo)
global world; % world with obstacles

noOfObs = size(world,1);

% find distance to bounding sphere
% position of the bounding sphere
  xBnd = world(1,1);
  yBnd = world(1,2);
  rBnd = world(1,3);
dist = (-1 * (findSqDistance(robo,[xBnd yBnd]))^2) + rBnd^2;

% find the rest of the obstacle distances
for i = 2:noOfObs
    xObs = world(i,1);
    yObs = world(i,2);
    rObs = world(i,3);
    dist = dist * ((findSqDistance(robo, [xObs yObs]))^2 - rObs^2);    
end
end
function dobsdot = findDerDobs(robo)

dobsdot = [0;0];
global world; % world with obstacles

noOfObs = size(world,1);

% find der. to bounding sphere
% position of the bounding sphere
  xBnd = world(1,1);
  yBnd = world(1,2);
  
for i=1:noOfObs
   
   xObs = world(i,1);
   yObs = world(i,2);
   
   dx = 1;
   dy = 1;
   
    if (i == 1)
        xTop = -2 * ( robo(1) - xObs);
        yTop = -2 * ( robo(2) - yObs);
        
        for j=1:noOfObs
            
            xObs = world(j,1);
            yObs = world(j,2);
            rObs = world(j,3);

            if ( j ~= i)
               if ( j == 1) % bounding sphere
                    dx = dx * (-1 * (findSqDistance(robo,[xObs yObs]))^2) + rObs^2;
                    dy = dy * (-1 * (findSqDistance(robo,[xObs yObs]))^2) + rObs^2; 
               else % other obstacles
                    dx = dx * ((findSqDistance(robo, [xObs yObs]))^2 - rObs^2);
                    dy = dy * ((findSqDistance(robo, [xObs yObs]))^2 - rObs^2); 
               end
            end  
            
        end
    else
        xTop = 2 * ( robo(1) - xObs);
        yTop = 2 * ( robo(2) - yObs);
        
        for j=1:noOfObs
            
            xObs = world(j,1);
            yObs = world(j,2);
            rObs = world(j,3);

            if ( j ~= i)
               if ( j == 1) % bounding sphere
                    dx = dx * (-1 * (findSqDistance(robo,[xObs yObs]))^2) + rObs^2;
                    dy = dy * (-1 * (findSqDistance(robo,[xObs yObs]))^2) + rObs^2; 
               else % other obstacles
                    dx = dx * ((findSqDistance(robo, [xObs yObs]))^2 - rObs^2);
                    dy = dy * ((findSqDistance(robo, [xObs yObs]))^2 - rObs^2); 
               end
            end  
            
        end

    end
    
    xTop = xTop * dx;
    yTop = yTop * dy;
    
    dobsdot(1) = dobsdot(1) + xTop;
    dobsdot(2) = dobsdot(2) + yTop;
end

end
function res = checkBoundary(pt)
global world;
% get the properties of the bounding sphere
xBnd = world(1,1);
yBnd = world(1,2);
rBnd = world(1,3);

% returns 1 if point is out of boundary
res = ( findSqDistance(pt,[xBnd yBnd]) > rBnd ) * 1;
end
function res = checkObstacle(pt)

global world;
% get the properties of the bounding sphere
noOfObs = size(world,1);

res = 1;

for i = 2:noOfObs
    xObs = world(i,1);
    yObs = world(i,2);
    rObs = world(i,3);
    
    % res remains 1 if pt is out of all obstacles
    res = res * ( findSqDistance(pt,[xObs yObs]) > rObs ) * 1;
end
end
% %% repulsive potential
% xRs = -worldSize:fres:worldSize ;
% yRs = -worldSize:fres:worldSize ;
% zRs =  zeros(size(yRs,2),size(xRs,2));
% [qx,qy]=meshgrid(-worldSize:fres:worldSize);
% for i=1:size(xRs,2)
%     for j=1:size(yRs,2)
%        zRep(j,i) = findObsDistance([xRs(i) yRs(j)]);                 
%     end
% end
% figure
% contourf(qx,qy,zRep,30)
% %colormap(hot)
% title('Repulsive Navigation Function Contours')
% figure 
% surf(qx,qy,zRep)
% figure
% quiver(xPs,yPs,zPs,zPs)
% title('Total Potential Navigation Function Vector Field, K=3')
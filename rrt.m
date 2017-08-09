%% Arena initialization 
clc,clear,close all
x_max = 40;
y_max = 30;
alpha=0:pi/20:2*pi;    % angle[0,2*pi] 
R=7;                   % diameter

position = [17 22];
x=R*cos(alpha)+position(1); 
y=R*sin(alpha)+position(2);
fill(x,y,'r');
figure(1);
hold on;
xlabel('x axis');
ylabel('y axis');
title('Grid map');
xp=1*cos(alpha)+position(1); 
yp=1*sin(alpha)+position(2); 
fill(xp,yp,'k');
xp=0.4*cos(alpha)+20; 
yp=0.4*sin(alpha)+15;
fill(xp,yp,'k');
xp=0.4*cos(alpha)+20; 
yp=0.4*sin(alpha)+29.6;
fill(xp,yp,'g');
axis([0 x_max 0 y_max]);
%% Variables initialization
x_list = [];
y_list = [];

EPS = 2;
numNodes = 100;

q_start.coord= [20 15];
q_start.cost = 0;
q_start.parent=0;
q_goal.coord = [20 30];
q_goal.cost = 0;
nodes(1) = q_start;
tic;

%% Main loop
for i =1:1:numNodes
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max/2+y_max/2)];
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410]);
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
   [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);
    if collision(q_rand, q_near.coord, position,R)
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
        % Within a radius of r, find all existing nodes
        q_nearest = [];
        r = 2;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if collision(nodes(j).coord, q_new.coord, position,R) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        for k = 1:1:length(q_nearest)
            if collision(q_nearest(k).coord, q_new.coord, position,R) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                hold on
            end
        end
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
route = [];
while q_end.parent ~= 0
    start = q_end.parent;
    route = [route q_end.coord'];
    plot([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], '--r', 'LineWidth', 4);
    hold on
    q_end = nodes(start);
end
toc


route = [route q_start.coord']
tine=[q_goal.coord' q_goal.coord'];

%% Routine cutting
for i=1:1:length(route)-2
   A=route(:,i)';
   B=route(:,i+2)';
   C = route(:,i+1)';
   if collision(tine(:,length(tine)-1),B,position,R)==1
       tine=[tine(:,1:length(tine)-1) B'];
   else
       tine=[tine C'];
   end
end

tine = [tine q_start.coord'];
pause(0.5)
plot(tine(1,:),tine(2,:),'-.b','LineWidth', 4)
half=[q_goal.coord'];

%% Route smoothing
for i=1:length(tine)-1
    half
    if collision(half(:,end),(tine(:,i+1)+tine(:,i))/2,position,R)==1
        half=[half (tine(:,i+1)+tine(:,i))/2]; 
    else
        half=[half tine(:,i)]; 
    end
end

half=[half q_start.coord']
pause(0.5)
plot(half(1,:),half(2,:),'--c','LineWidth', 4)
hold on;
fill(xp,yp,'k');
axis([0 x_max 0 y_max]);

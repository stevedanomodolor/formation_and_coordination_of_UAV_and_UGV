%% test the clustering algorithm
close all
clear all


%  Generate a square formation 
X = [-1 1;
    1 1;
    1 0;
    -1 0];

%  Generate a triangular formation 
y = 1;
x = y/atan(deg2rad(60));
X = [0 y;
    x 0;
    -x 0];

%  Generate a pentagon formation 
l = 1;
ex = (l/2)+l*cos(deg2rad(72));
ey = l*sin(deg2rad(72));
a3 = [l/2,0];
a4 = [-l/2 0];
a2 = [ex,ey ];
a5 = [-ex,ey];
a1 = [0, ey+sqrt(l*l-ex*ex)];
X = [a1;a2;a3;a4;a5];
X_ = X';
ugv_loc = [ 1 -1; -1 -1];
ugv_loc_ = ugv_loc';
% [idx,C] = kmeans(X,2);


[idx,C] = compute_cluster(X_,2,ugv_loc_);


figure;
plot(ugv_loc(:,1),ugv_loc(:,2),'r*', 'MarkerSize',12)
hold on
plot(X(:,1),X(:,2),'r*', 'MarkerSize',12)
hold on
plot(X(idx==1,1),X(idx==1,2),'r.','MarkerSize',12)
hold on
plot(X(idx==2,1),X(idx==2,2),'b.','MarkerSize',12)
plot(C(:,1),C(:,2),'kx',...
     'MarkerSize',15,'LineWidth',3) 
legend('ugv_location','origin point', 'Cluster 1','Cluster 2','Centroids',...
       'Location','NW')
title 'Cluster Assignments and Centroids'
hold off


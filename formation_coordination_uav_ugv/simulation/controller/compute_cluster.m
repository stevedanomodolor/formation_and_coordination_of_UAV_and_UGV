function [indx, centroids] = compute_cluster(dl, n, ml)
% dl location of the drones 
% n --> number of mobile robot
% ml ---> mobile robot locations
% uses the kmeans algorithms 

% convert matrix format to format of kmeans 
% The reason why the initial location of the ugv is used is because 
% this way the centroid is always fix, if not the centroids changes 
dl = dl';

ml = ml';

[indx, centroids] = kmeans(dl, n, 'Start',ml);






end
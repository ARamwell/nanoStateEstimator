function [R, t, scale] =alignTraj(points1, points2, isSimilarity)
%computeTransform computes a rigid or similarity transformation between two  
%  sets of 3-D points. points1 and points2 must be M-by-3 arrays of [x,y,z] 
%  coordinates.
%
% Umeyama, Shinji. "Least-squares estimation of transformation parameters 
% between two point patterns." IEEE Transactions on Pattern Analysis & 
% Machine Intelligence 13, no. 04 (1991): 376-380

numPoints = size(points1, 1);

% Find data centroid and deviations from centroid
centroid1 = mean(points1);  % equation 34
centroid2 = mean(points2);  % equation 35

normPoints1 = bsxfun(@minus, points1, centroid1);
normPoints2 = bsxfun(@minus, points2, centroid2);

% Covariance matrix
C = normPoints1'*normPoints2/numPoints;  % equation 38

[U,S,V] = svd(C);

% Handle the reflection case
D = diag([1 1 sign(det(U*V'))]); % equation 39

% Compute rotation
R = V*D*U'; % equation 40

% Compute scale
if isSimilarity
    signa_x = mean( sum(normPoints1.^2, 2) ); % equation 36
    scale = trace(S*D)/signa_x; % equation 42
else
    scale = 1;
end

% Compute the translation
t = centroid2' - scale * R*centroid1'; % equation 41
end



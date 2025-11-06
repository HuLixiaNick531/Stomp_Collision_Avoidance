% Input: 
%   nSamplePaths: the number of sample paths 
%   sigma： sample covariance matrix
%   theta: mean trajectory from last iteration
% Output:
%   theta_paths: sampled trajectories
%   em: sampled Gaussian trajectory for each joint

function [theta_paths, em]=stompSamples(nSamplePaths,sigma,theta)
% Sample theta (joints angles) trajectory

[nJoints, nDiscretize] = size(theta);

em = cell(1,nJoints);
ek = cell(1,nSamplePaths);

theta_paths = cell(1, nSamplePaths);
mu=zeros(1,length(sigma));

%% TODO: complete code of independent sampling for each joint
for m = 1 : nJoints
    % Each joint is sampled independently.
    % The starting q0 and final qT are fixed, so set the sample to 0.
    % Sample from multivariable Gaussian distribution
    em{m} = zeros(nSamplePaths, nDiscretize);
    if nDiscretize > 2
        z = mvnrnd(zeros(1, size(sigma,1)), sigma, nSamplePaths); % nSamplePaths x (nDiscretize-2)
        em{m}(:, 2:nDiscretize-1) = z;
    end
end

%% Regroup it by samples
emk = [em{:}];
for k=1:nSamplePaths
    ek{k} = reshape(emk(k,:),nDiscretize, nJoints)';
    theta_paths{k} = theta + ek{k};
    
end
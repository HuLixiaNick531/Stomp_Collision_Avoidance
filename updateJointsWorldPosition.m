%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta)
%UPDATEJOINTSWORLDPOSITION Forward kinematics via PoE/space screws.
%   This implementation replaces getTransform with the Product of
%   Exponentials formulation using the Modern Robotics toolbox helpers.

theta = double(theta(:));
nJoints = numel(theta);

persistent cachedRobot poeCache modernRoboticsPathAdded
if isempty(modernRoboticsPathAdded)
    % Make sure the Modern Robotics utilities are on the MATLAB path once.
    mrPath = fullfile(fileparts(mfilename('fullpath')), 'modern_robotics');
    if exist(mrPath, 'dir')
        addpath(mrPath);
    else
        error('Modern Robotics helper directory not found: %s', mrPath);
    end
    modernRoboticsPathAdded = true;
end

if isempty(cachedRobot) || ~isequal(cachedRobot, robot_struct)
    poeCache = buildPoECache(robot_struct);
    cachedRobot = robot_struct;
end

if nJoints ~= poeCache.numJoints
    error('Joint vector length (%d) does not match robot DOF (%d).', ...
          nJoints, poeCache.numJoints);
end

T = cell(1, nJoints);
X = zeros(nJoints, 4);

for k = 1:nJoints
    T{k} = FKinSpace(poeCache.Mlist{k}, poeCache.Slist(:, 1:k), theta(1:k));
    X(k, :) = (T{k} * [0; 0; 0; 1])';
end

end

function cache = buildPoECache(robot_struct)
% buildPoECache precomputes screw axes and home transformations.

bodyNames = cellstr(robot_struct.BodyNames);
nBodies = numel(bodyNames);
nameToIndex = containers.Map(bodyNames, num2cell(1:nBodies));
baseName = char(robot_struct.BaseName);
baseToBodyHome = cell(1, nBodies);

homeConfig = robot_struct.homeConfiguration;
nActive = numel(homeConfig);

Slist = zeros(6, nActive);
Mlist = cell(1, nActive);
activeBodyNames = cell(1, nActive);
activeIdx = 0;

for k = 1:nBodies
    body = robot_struct.Bodies{k};
    joint = body.Joint;
    parentName = body.Parent;

    if isa(parentName, 'rigidBody')
        parentKey = parentName.Name;
    else
        parentKey = char(parentName);
    end
    if strcmp(parentKey, baseName)
        T_parent = eye(4);
    else
        parentIdx = nameToIndex(parentKey);
        T_parent = baseToBodyHome{parentIdx};
    end

    T_joint = T_parent * joint.JointToParentTransform;
    T_body = T_joint / joint.ChildToJointTransform;
    baseToBodyHome{k} = T_body;

    if strcmp(joint.Type, 'fixed')
        continue;
    end

    activeIdx = activeIdx + 1;
    activeBodyNames{activeIdx} = bodyNames{k};
    axis_local = joint.JointAxis(:);
    R_base_joint = T_joint(1:3, 1:3);

    switch joint.Type
        case {'revolute', 'continuous'}
            w = R_base_joint * axis_local;
            q = T_joint(1:3, 4);
            v = -cross(w, q);
            Slist(:, activeIdx) = [w; v];
        case 'prismatic'
            v = R_base_joint * axis_local;
            Slist(:, activeIdx) = [zeros(3, 1); v];
        otherwise
            error('Unsupported joint type "%s" for body %s.', joint.Type, body.Name);
    end

    Mlist{activeIdx} = T_body;
end

if activeIdx ~= nActive
    error(['Mismatch between detected active joints (%d) and the home ' ...
           'configuration length (%d).'], activeIdx, nActive);
end

cache = struct( ...
    'Slist', Slist, ...
    'Mlist', {Mlist}, ...
    'bodyNames', {activeBodyNames}, ...
    'numJoints', activeIdx);

end


% % forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame



% function [X, T] = updateJointsWorldPosition(robot_struct, theta)

% % In this sample code, we directly call the MATLAB built-in function getTransform
% % to calculate the forward kinemetics
    

% % Update the robot configuration structure used by Matlab
% % Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
% theta_cell = num2cell(theta);
% % Because the getTranform() function can only takes in structure array
% % robot Configuration, we have to construct it first by copying the structure from the homeConfiguration
% % robot_struct = loadrobot(robot_name); % by default, it loads the robot with structure data format
% tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
% [tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta
% % get the number of joints
% nJoints = length(theta);
% T = cell(1,nJoints);
% X = zeros(nJoints, 4); 

% for k=1:nJoints
%     % get the homegeneous transformation from kth joint's frame to the
%     % base frame
%     % Use the Matlab built-in function getTransfrom to obtain the pose T
%     % getTransform can only takes in structure array Configuration
%     %% TODO:
%     T{k} = getTransform(robot_struct, tConfiguration, robot_struct.BodyNames{k});
%     %% TODO: Get joint's world coordinates
%     X(k,:) = (T{k} * [0; 0; 0; 1])';
    
% end
    
% end

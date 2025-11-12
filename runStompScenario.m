%% End-to-end STOMP planning pipeline for the KINOVA example
% This script replicates the Live Script setup before calling helperSTOMP.

renderFigHandle = [];
renderAxHandle = [];
iterLabelHandle = [];
if exist('stompRenderFigure','var')
    renderFigHandle = stompRenderFigure;
end
if exist('stompRenderAxes','var')
    renderAxHandle = stompRenderAxes;
end
if exist('stompIterLabel','var')
    iterLabelHandle = stompIterLabel;
end

clearvars -except renderFigHandle renderAxHandle iterLabelHandle

stompRenderFigure = renderFigHandle;
stompRenderAxes = renderAxHandle;
stompIterLabel = iterLabelHandle;
clear renderFigHandle renderAxHandle iterLabelHandle;

% parameters
use_default_robot = false;
if ~use_default_robot
    robot_name = 'universalUR5e'; % name of robot
    endEffector = "tool0";        % name of end effector
end

% Load robot model
if use_default_robot
    robot_name = 'kinovaGen3';
    robot = loadrobot(robot_name, 'DataFormat', 'column');
else
    robot = loadrobot(robot_name, 'DataFormat', 'column');
end

% Robot metadata
numJoints = numel(homeConfiguration(robot));
if use_default_robot
    endEffector = "EndEffector_Link";
end

% Initial end-effector pose
taskInit = trvec2tform([0.4 0 0.2]) * axang2tform([0 1 0 pi]);

% Inverse kinematics for start configuration
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];
currentRobotJConfig = ik(endEffector, taskInit, weights, robot.homeConfiguration);
currentRobotJConfig = wrapToPi(currentRobotJConfig);

% Final end-effector pose and configuration
taskFinal = trvec2tform([0.35 0.55 0.35]) * axang2tform([0 1 0 pi]);
anglesFinal = rotm2eul(taskFinal(1:3,1:3), 'XYZ');
poseFinal = [taskFinal(1:3,4); anglesFinal']; % [x y z phi theta psi]
finalRobotJConfig = ik(endEffector, taskFinal, weights, currentRobotJConfig);
finalRobotJConfig = wrapToPi(finalRobotJConfig);

% Obstacles and visualization helpers
helperCreateObstaclesKINOVA;
x0 = [currentRobotJConfig', zeros(1,numJoints)];
helperInitialVisualizerKINOVA;
safetyDistance = 0.01;

% Run STOMP planner
helperSTOMP;

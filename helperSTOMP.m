%Parameters
% T = 5; 
nDiscretize = 50; % number of discretized waypoint
nPaths = 100; % number of sample paths
convergenceThreshold = 0.05; % convergence threshhold

% Initial guess of joint angles theta is just linear interpolation of q0
% and qT
q0 = currentRobotJConfig;
qT = finalRobotJConfig;
numJoints = length(q0);
theta=zeros(numJoints, nDiscretize);
for k=1:length(q0)
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end

% by default, it loads the robot with the structure data format
robot_struct = loadrobot(robot_name); 

% store sampled paths
theta_samples = cell(1,nPaths);

useSharedRender = exist('stompRenderAxes','var') && isgraphics(stompRenderAxes);
if useSharedRender
    renderAxes = stompRenderAxes;
    renderFigure = ancestor(renderAxes,'figure');
    showArgs = {'PreservePlot', false, 'Frames', 'on', 'Parent', renderAxes};
else
    renderAxes = [];
    renderFigure = [];
    showArgs = {'PreservePlot', false, 'Frames', 'on'};
end

if exist('stompIterLabel','var') && isgraphics(stompIterLabel)
    iterStatusLabel = stompIterLabel;
else
    iterStatusLabel = [];
end

%% for calculating the acceleration of theta
% Precompute
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1); 
R = A' * A;
Rinv = inv(R);
% The smoothing matrix M, normalized from Rinv by each column, no longer symmetric
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1); 
% R inverse is normalized so that the exploration is controlled to have samples within the created voxel world
Rinv = 1.5*Rinv/sum(sum(Rinv)); 


%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
QthetaOld = 0;
overallTimer = tic;
iterTimer = tic;
updateIterLabel(iterStatusLabel, 0, Qtheta, 0, 0);

iter=0;
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1;
    % overall cost: Qtheta
    QthetaOld = Qtheta;
    % use tic and toc for printing out the running time
    tic
    %% TODO: Complete the following code. The needed functions are already given or partially given in the folder.
    %% TODO: Sample noisy trajectories
    [theta_samples, em] = stompSamples(nPaths, Rinv, theta);
    %% TODO: Calculate Local trajectory cost for each sampled trajectory
    % variable declaration (holder for the cost):
    Stheta = zeros(nPaths, nDiscretize);
    for k = 1:nPaths
        [Stheta_k, ~] = stompTrajCost(robot_struct, theta_samples{k}, R, voxel_world);
        Stheta(k, :) = Stheta_k;
    end
    
    %% TODO: Given the local traj cost, update local trajectory probability
    trajProb = stompUpdateProb(Stheta);
    
    %% TODO: Compute delta theta (aka gradient estimator, the improvement of the delta)
    dtheta = stompDTheta(trajProb, em);
    [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M);

    %% TODO: Compute the cost of the new trajectory
    [~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
    iterElapsed = toc(iterTimer);
    totalElapsed = toc(overallTimer);
    updateIterLabel(iterStatusLabel, iter, Qtheta, iterElapsed, totalElapsed);
    iterTimer = tic;
 
    toc

    Q_time = [Q_time Qtheta];
    % control cost
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta % display overall cost
    RAR  % display control cost

    % Record the itermediate training trajectories for later animation
    theta_animation{iter}=theta;

    % Set the stopping iteration criteria:
    if iter > 50 
        disp('Maximum iteration (50) has reached.')
        break
    end

    if sum(dtheta_smoothed,'all') == 0
    disp('Estimated gradient is 0 and Theta is not updated: there could be no obstacle at all')
    break
    end

end

iterElapsed = toc(iterTimer);
totalElapsed = toc(overallTimer);
updateIterLabel(iterStatusLabel, iter, Qtheta, iterElapsed, totalElapsed);

disp('STOMP Finished.');






%% check collision
inCollision = false(nDiscretize, 1); % initialization the collision status vector
worldCollisionPairIdx = cell(nDiscretize,1); % Initialization: Provide the bodies that are in collision

for i = 1:nDiscretize

    [inCollision(i),sepDist] = checkCollision(robot,theta(:,i),world,"IgnoreSelfCollision","on","Exhaustive","on");


    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
    worldCollidingPairs = [bodyIdx,worldCollisionObjIdx];
    worldCollisionPairIdx{i} = worldCollidingPairs;

end
% Display whether there is collision:
isTrajectoryInCollision = any(inCollision)


%% Record the whole training/learning process in video file
enableVideoTraining = 0;



v = VideoWriter('KinvaGen3_Training.avi');
v.FrameRate = 15;
open(v);

if useSharedRender
    htext = text(renderAxes,-0.2,0.6,0.7,'Iteration = 0','HorizontalAlignment','left','FontSize',14);
else
    htext = text(-0.2,0.6,0.7,'Iteration = 0','HorizontalAlignment','left','FontSize',14);
    renderFigure = gcf;
end

if enableVideoTraining == 1
    theta_animation_tmp = theta_animation(~cellfun('isempty',theta_animation));
    nTraining = length(theta_animation_tmp);
    for k=0:5:nTraining
        
        UpdatedText = ['Iteration = ',num2str(k)];
        set(htext,'String',UpdatedText)
        theta_tmp = theta_animation_tmp{k+1};

        for t=1:size(theta_tmp,2)
            show(robot, theta_tmp(:,t), showArgs{:});
            frame = captureStompFrame(renderFigure);
            writeVideo(v,frame);
%             pause(1/15);
            %     pause;
        end
        pause(1/15);
    end
end
close(v);



%% Record planned trajectory to video files
enableVideo = 0;
if enableVideo == 1
    v = VideoWriter('KinvaGen3_wEEConY3.avi');
    v.FrameRate =2;
    open(v);

    for t=1:size(theta,2)
        show(robot, theta(:,t), showArgs{:});
        drawnow;
        frame = captureStompFrame(renderFigure);
        writeVideo(v,frame);
        pause(5/20);
        %     pause;
    end
    close(v);
end
%% Show the planned trajectory
displayAnimation = 1;
if displayAnimation

    % 计算末端执行器在每个离散时刻的位姿，并画出轨迹
    eePos = zeros(3, size(theta,2));
    for t = 1:size(theta,2)
        T = getTransform(robot, theta(:,t), "tool0");  % 用你的末端连杆名
        eePos(:,t) = tform2trvec(T).';                         % 转成 xyz
    end
    if useSharedRender
        axDisplay = renderAxes;
    else
        axDisplay = gca;
    end
    % hold(axDisplay,'on');
    plot3(axDisplay, eePos(1,:), eePos(2,:), eePos(3,:), 'r-', 'LineWidth', 1.5); % 规划轨迹
    scatter3(axDisplay, eePos(1,1), eePos(2,1), eePos(3,1), 70, 'g', 'filled', 'Marker','o');   % 起点
    scatter3(axDisplay, eePos(1,end), eePos(2,end), eePos(3,end), 70, 'r', 'filled', 'Marker','s'); % 终点
    % hold(axDisplay,'off');
    frame = captureStompFrame(renderFigure);
    
    for t=1:size(theta,2)
        show(robot, theta(:,t), showArgs{:});
        drawnow;
        frame = captureStompFrame(renderFigure);
        pause(5/20);
        %     pause;
    end
end



%% save data
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')

function frame = captureStompFrame(renderFigure)
if isempty(renderFigure) || ~isgraphics(renderFigure)
    renderFigure = gcf;
end
drawnow;
frame = getframe(renderFigure);
end

function updateIterLabel(labelHandle, iter, cost, iterElapsed, totalElapsed)
if isempty(labelHandle) || ~isgraphics(labelHandle)
    return;
end
labelHandle.String = sprintf('Iter: %d, Cost: %.3f, Iter Time: %.2fs, Total: %.2fs', ...
    iter, cost, iterElapsed, totalElapsed);
drawnow limitrate;
end


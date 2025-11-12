% Given a trajectory, calculate its cost
function [Stheta, Qtheta] = stompTrajCost(robot_struct, theta,  R, voxel_world)
% Compute the local trajectory cost at each discretization theta point, as 
% well as the overall trajecotry cost (the Qtheta)

% Costi = stompCompute_Cost(robot, theta, Env);
% Compute the cost of all discretized points on one trajectory
[~, nDiscretize] = size(theta);
% Obstacle cost
qo_cost = zeros(1, nDiscretize);
% Constraint costs
qc_cost = zeros(1, nDiscretize);

% Get the coordinates of joints in World frame 
[X, T] = updateJointsWorldPosition(robot_struct, theta(:, 1));
% Construct the spheres around the robot manipulator for collision
% avoidance
[sphere_centers,radi] = stompRobotSphere(X);
% Initial velocity at the sphere centers around the manipulator is 0
vel = zeros(length(sphere_centers), 1);
qo_cost(1) = stompObstacleCost(sphere_centers,radi, voxel_world, vel);

% Define qc_cost to add constraint on the end-effector
w_qc = 1;
v_pose_z = T{end}(1:3, 3);
v_world_z = [0; 0; 1];
dot_product = dot(v_pose_z, v_world_z);
qc_cost(1) = w_qc * (max(-1.0, min(1.0, dot_product)));

for i = 2 : nDiscretize
    sphere_centers_prev = sphere_centers;
    % Calculate the kinematics of the manipulator, given the
    % configuration theta values at different time (i=2:nDiscretize)
    [X, T] = updateJointsWorldPosition(robot_struct, theta(:, i));
    [sphere_centers, radi] = stompRobotSphere(X);
    % xb: 3D workspace position of sphere b at the current time
    % Approximate the speed (xb_dot) using the finite difference of the current and
    % the previous position
    minCenters = min(size(sphere_centers_prev,1), size(sphere_centers,1));
    vel = zeros(size(sphere_centers,1), 1);
    if minCenters > 0
        vel(1:minCenters) = vecnorm( ...
            sphere_centers_prev(1:minCenters, :) - sphere_centers(1:minCenters, :), ...
            2, 2);
    end
    qo_cost(i) = stompObstacleCost(sphere_centers,radi, voxel_world, vel);
    
    % Define qc_cost to add constraint on the end-effector
    v_pose_z = T{end}(1:3, 3);
    v_world_z = [0; 0; 1];
    dot_product = dot(v_pose_z, v_world_z);
    qc_cost(i) = w_qc * (max(-1.0, min(1.0, dot_product)));
end

%% Local trajectory cost: you need to specify the relative weights between different costs
Stheta = 1000*qo_cost + qc_cost;

% sum over time and add the smoothness cost
theta = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 1/2 * sum(theta * R * theta', "all");

end


    

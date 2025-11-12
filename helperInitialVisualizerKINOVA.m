% Visualize the obstacles and robot manipulator

% Initial visualizer 
positions = x0(1:numJoints)';

useSharedRender = exist('stompRenderAxes','var') && isgraphics(stompRenderAxes);
if useSharedRender
    ax1 = stompRenderAxes;
    renderFig = ancestor(ax1,'figure');
    cla(ax1);
else
    renderFig = figure('Position', [375 446 641 480]);
    ax1 = axes('Parent', renderFig);
end

show(robot, positions(:,1),'PreservePlot', false, 'Frames', 'off','Parent', ax1);
view(ax1,150,29)
hold(ax1,'on')
axis(ax1,[-0.8 0.8 -0.6 0.7 -0.2 0.7]);
plot3(ax1, poseFinal(1), poseFinal(2), poseFinal(3),'r.','MarkerSize',20)

for i=1:numel(world)
    [~,pObj] = show(world{i},'Parent',ax1);
    pObj.LineStyle = 'none';
end





function launchStompGUI()
% launchStompGUI Create a classic figure-based GUI with embedded axes to run STOMP.

fig = figure('Name','STOMP Planner','NumberTitle','off','MenuBar','none',...
    'ToolBar','figure','Color',[1 1 1],'Position',[200 200 720 520]);

renderAx = axes('Parent',fig,'Units','pixels','Position',[60 180 600 320]);
view(renderAx,135,30);
grid(renderAx,'on');
xlabel(renderAx,'X (m)');
ylabel(renderAx,'Y (m)');
zlabel(renderAx,'Z (m)');
title(renderAx,'Trajectory / Training Preview');

statusTxt = uicontrol(fig,'Style','text','String','Ready','FontSize',11,...
    'HorizontalAlignment','center','BackgroundColor',[1 1 1],...
    'Position',[180 40 360 30]);

iterLbl = uicontrol(fig,'Style','text','String','Iter: -, Cost: -, Time: -',...
    'FontSize',11,'HorizontalAlignment','center','BackgroundColor',[1 1 1],...
    'Position',[180 70 360 25]);

btnW = 140; btnH = 35; y = 110;
uicontrol(fig,'Style','pushbutton','String','Init Visualizer',...
    'FontSize',11,'Position',[190 y btnW btnH],...
    'Callback',@(src,evt)runHelperCommand(src,statusTxt,iterLbl,renderAx,...
        'x0 = [currentRobotJConfig'', zeros(1,numJoints)]; helperInitialVisualizerKINOVA;',...
        'Visualizer ready'));
uicontrol(fig,'Style','pushbutton','String','Run STOMP',...
    'FontSize',12,'Position',[370 y btnW btnH],...
    'Callback',@(src,evt)runHelperCommand(src,statusTxt,iterLbl,renderAx,...
        'helperSTOMP;', 'STOMP finished'));

assignin('base','stompRenderFigure',fig);
assignin('base','stompRenderAxes',renderAx);
assignin('base','stompIterLabel',iterLbl);

end

function runHelperCommand(src,statusTxt,iterLbl,renderAx,cmd,successMsg)
statusTxt.String = 'Running...';
src.Enable = 'off';
drawnow;

try
    assignin('base','stompRenderFigure',ancestor(renderAx,'figure'));
    assignin('base','stompRenderAxes',renderAx);
    assignin('base','stompIterLabel',iterLbl);
    evalin('base',cmd);
    statusTxt.String = successMsg;
catch ME
    statusTxt.String = 'Failed - see details';
    warndlg(ME.message,'STOMP Error','modal');
end

src.Enable = 'on';
end

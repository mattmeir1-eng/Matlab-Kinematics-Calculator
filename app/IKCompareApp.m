% Uses standard Figure to prevent Robotics Toolbox UIAxes errors.
function IKCompareApp
    addpath(genpath('src'));
    if ~exist('data/artifacts.mat', 'file')
        errordlg('Please run "run_all.m" first.', 'Missing Data'); return;
    end
    loaded = load('data/artifacts.mat');
    robot = loaded.robot; L = loaded.L; limits = loaded.limits;
    netData = load('data/net.mat');
    net = netData.net;
    qHome = [0 0 0];
    currentQ = qHome;
    %UI
    fig = figure('Name', '3-DOF IK Benchmarker', ...
        'NumberTitle', 'off', ...
        'Color', [0.15 0.15 0.15], ...
        'Position', [100 100 1000 600], ...
        'MenuBar', 'none', ...
        'ToolBar', 'none');

    %axes
    ax = axes('Parent', fig, ...
        'Units', 'normalized', ...
        'Position', [0.05 0.1 0.55 0.85], ... 
        'Color', [0.2 0.2 0.2], ...
        'XColor', [0.8 0.8 0.8], 'YColor', [0.8 0.8 0.8], 'ZColor', [0.8 0.8 0.8], ...
        'GridColor', 'w');
    grid(ax, 'on'); view(ax, 3); axis(ax, 'equal'); hold(ax, 'on');
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');

    %robot
    show(robot, currentQ, 'Parent', ax, 'PreservePlot', false);
    targetPlot = plot3(ax, 0, 0, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
    set(targetPlot, 'Visible', 'off');

    %controls
    pnl = uipanel('Parent', fig, ...
        'Title', 'Control Panel', ...
        'Units', 'normalized', ...
        'Position', [0.65 0.05 0.3 0.9], ...
        'BackgroundColor', [0.25 0.25 0.25], ...
        'ForegroundColor', 'w', ...
        'FontSize', 12);

    %helpers
    makeLbl = @(txt, pos) uicontrol(pnl, 'Style','text', 'String',txt, ...
        'Units','normalized', 'Position',pos, ...
        'BackgroundColor',[0.25 0.25 0.25], 'ForegroundColor','w', ...
        'HorizontalAlignment','left', 'FontSize',10);
    makeEdit = @(val, pos) uicontrol(pnl, 'Style','edit', 'String',num2str(val), ...
        'Units','normalized', 'Position',pos, ...
        'BackgroundColor',[0.1 0.1 0.1], 'ForegroundColor','w', 'FontSize',10);

    %inputs
    makeLbl('X (m):', [0.1 0.85 0.3 0.05]); hX = makeEdit(0.25, [0.4 0.85 0.5 0.05]);
    makeLbl('Y (m):', [0.1 0.78 0.3 0.05]); hY = makeEdit(0.00, [0.4 0.78 0.5 0.05]);
    makeLbl('Z (m):', [0.1 0.71 0.3 0.05]); hZ = makeEdit(0.15, [0.4 0.71 0.5 0.05]);

    makeLbl('Method:', [0.1 0.60 0.8 0.05]);
    hMethod = uicontrol(pnl, 'Style', 'popupmenu', ...
        'String', {'Neural Network', 'Closed-Form Analytic', 'Numeric (Iterative)'}, ...
        'Units', 'normalized', 'Position', [0.1 0.55 0.8 0.05], 'FontSize', 10);

    hAnim = uicontrol(pnl, 'Style', 'checkbox', 'String', 'Animate', ...
        'Units', 'normalized', 'Position', [0.1 0.48 0.8 0.05], ...
        'BackgroundColor', [0.25 0.25 0.25], 'ForegroundColor', 'w', ...
        'Value', 1, 'FontSize', 10);

    %button
    uicontrol(pnl, 'Style', 'pushbutton', 'String', 'SOLVE & MOVE', ...
        'Units', 'normalized', 'Position', [0.1 0.35 0.8 0.1], ...
        'BackgroundColor', [0 0.45 0.74], 'ForegroundColor', 'w', ...
        'FontWeight', 'bold', 'FontSize', 12, ...
        'Callback', @onCompute);

    %metrics
    hTime = uicontrol(pnl, 'Style','text', 'String','Latency: -- ms', ...
        'Units','normalized', 'Position', [0.1 0.2 0.8 0.05], ...
        'BackgroundColor',[0.25 0.25 0.25], 'ForegroundColor',[0.4 1 0.4], ...
        'FontSize', 11, 'HorizontalAlignment','left');
    hErr = uicontrol(pnl, 'Style','text', 'String','Error: -- mm', ...
        'Units','normalized', 'Position', [0.1 0.15 0.8 0.05], ...
        'BackgroundColor',[0.25 0.25 0.25], 'ForegroundColor',[1 0.4 0.4], ...
        'FontSize', 11, 'HorizontalAlignment','left');
    
    %logic
    function onCompute(~,~)
        target = [str2double(hX.String), str2double(hY.String), str2double(hZ.String)];
        set(targetPlot, 'XData',target(1), 'YData',target(2), 'ZData',target(3), 'Visible','on');
        try
            switch hMethod.Value
                case 1, solveFun = @() predictNN(net, target);
                case 2, solveFun = @() solveAnalytic(target);
                case 3, solveFun = @() ik_numeric(robot, target, currentQ);
            end
            t = timeit(solveFun);
            qSol = solveFun();
            if isempty(qSol) || any(isnan(qSol))
                hTime.String = 'Status: Fail'; return; 
            end
            T = getTransform(robot, qSol, 'tool');
            err = 1000 * norm(tform2trvec(T) - target);
            hTime.String = sprintf('Latency: %.3f ms', t*1000);
            hErr.String  = sprintf('Error: %.3f mm', err);
            if hAnim.Value
                for a = linspace(0,1,15)
                    show(robot, currentQ + a*(qSol-currentQ), 'Parent', ax, 'PreservePlot', false);
                    drawnow limitrate;
                end
            end
            show(robot, qSol, 'Parent', ax, 'PreservePlot', false);
            currentQ = qSol;
        catch ME
            errordlg(ME.message);
        end
    end

    function q = solveAnalytic(t)
        [Qs,~] = ik_analytic(t, L, limits);
        if isempty(Qs), q=[]; return; end
        [~,idx] = min(vecnorm(Qs - currentQ, 2, 2));
        q = Qs(idx,:);
    end
end

    % 初始化PUMA560模型
    mdl_puma560;
    
    % 各关节角度范围（度数）
    q_limits_deg = [
        -160, 160;   % q1
        -225,  45;   % q2
        -45,  225;   % q3
        -110, 110;   % q4
        -100, 100;   % q5
        -266, 266;   % q6
    ];
    
    % 创建GUI窗口
    fig = figure('Name', 'PUMA560 Teach Pendant', 'NumberTitle', 'off',...
                 'Position', [100 100 1000 600], 'DeleteFcn', @closeFigure);
    
    % 创建坐标系用于机器人显示
    ax = axes('Parent', fig, 'Position', [0.4 0.1 0.55 0.8]);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    view(ax, 3);
    
    % 初始化控件
    sliderWidth = 200;
    ctrlY = 500;
    sliders = gobjects(6,1);
    jointDisp = gobjects(6,1);
    
    % 创建关节控制滑块
    for i = 1:6
        % 关节标签
        uicontrol('Style','text', 'Position',[20 ctrlY 80 20],...
                  'String',['Joint ',num2str(i)], 'HorizontalAlignment','left');
        
        % 滑块控件
        sliders(i) = uicontrol('Style','slider',...
            'Position',[120 ctrlY sliderWidth 20],...
            'Min',q_limits_deg(i,1), 'Max',q_limits_deg(i,2), 'Value',0,...
            'SliderStep',[1/diff(q_limits_deg(i,:)) 10/diff(q_limits_deg(i,:))],...
            'Callback',@updateRobot);
        
        % 角度显示
        jointDisp(i) = uicontrol('Style','text',...
            'Position',[sliderWidth+130 ctrlY 60 20],...
            'String','0.0°', 'HorizontalAlignment','left');
        
        ctrlY = ctrlY - 40;
    end
    
    % 末端执行器坐标显示
    posDisp = uicontrol('Style','text', 'Position',[20 100 350 60],...
        'String','Position: [0.000, 0.000, 0.000]',...
        'FontSize',10, 'HorizontalAlignment','left');
    
    % 姿态四元数显示
    orientDisp = uicontrol('Style','text', 'Position',[20 40 350 60],...
        'String','Orientation: [0.000, 0.000, 0.000, 0.000]',...
        'FontSize',10, 'HorizontalAlignment','left');
    
    % 存储句柄
    handles = struct('sliders',sliders, 'jointDisp',jointDisp,...
                    'posDisp',posDisp, 'orientDisp',orientDisp, 'ax',ax);
    guidata(fig, handles);
    
    % 初始更新
%     updateRobot();
    
    % 回调函数 - 更新机器人状态
    function updateRobot(~,~)
        handles = guidata(gcbf);
        
        % 获取当前关节角度（度数）
        q_deg = arrayfun(@(s) s.Value, handles.sliders);
        
        % 更新关节角度显示
        for i = 1:6
            handles.jointDisp(i).String = sprintf('%.1f°', q_deg(i));
        end
        
        % 转换角度单位并计算运动学
        q_rad = deg2rad(q_deg);
        T = p560.fkine(q_rad);
        
        % 更新末端执行器显示
        pos = transl(T);
        handles.posDisp.String = sprintf('Position: [%.3f, %.3f, %.3f]', pos);
        
        % 更新姿态显示（四元数）
        orient = quaternion(T.R, 'rotmat', 'frame');
        handles.orientDisp.String = sprintf('Orientation: [%.3f, %.3f, %.3f, %.3f]',...
            orient.a, orient.b, orient.c, orient.d);
        
        % 更新机器人图形
        cla(handles.ax);  % 清除上一帧
        p560.plot(q_rad, 'parent', handles.ax, 'noraise', 'nowrist');
        drawnow;
    end

    % 关闭回调
    function closeFigure(~,~)
        delete(fig);
        clear p560
    end

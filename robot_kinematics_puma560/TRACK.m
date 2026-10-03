clear; clc;
% 加载PUMA560模型
mdl_puma560;
robot = p560;
q0 = [0, 0, 0, 0, 0, 0]; % 初始关节角

% 圆心和半径（在机械臂工作空间内）
center = [0.6, 0, 0.2]; % [x, y, z]
radius = 0.1;
theta = linspace(0, 2*pi, 50); % 50个点

% 生成笛卡尔空间轨迹点
x = center(1) + radius*cos(theta);
y = center(2) + radius*sin(theta);
z = center(3)*ones(size(theta));
orient = repmat([0, pi/2, 0], length(theta), 1); % 固定姿态

%逆运动学求解
    % 预分配关节角度数组
    q_ik = zeros(length(theta), 6);
    for i = 1:length(theta)
        T = transl(x(i), y(i), z(i)) * eul2tr(orient(i,:));
        q_ik(i,:) = robot.ikine6s(T); % 使用解析逆解
    end

%三次多项式规划
    % 时间参数
    t = linspace(0, 10, length(theta));
    t_interp = linspace(0, 10, 200);
    
    % 各关节三次样条插值
    q_cubic = zeros(length(t_interp), 6);
    for j = 1:6
        q_cubic(:,j) = interp1(t, q_ik(:,j), t_interp, 'spline');
    end

%五次多项式规划
    % 使用Robotics Toolbox内置函数
    q_quintic = [];
for i = 1:size(q_ik, 1)-1
    [q_seg, ~, ~] = jtraj(q_ik(i,:), q_ik(i+1,:), linspace(0, 1, 10));
    q_quintic = [q_quintic; q_seg];
end


% 逆运动学求解
q_nurbs = zeros(length(x), 6);
for i = 2:length(x)-1
    T = transl(x(i), y(i), z(i)) * eul2tr([0, pi/2, 0]); % 固定姿态
    q_nurbs(i,:) = robot.ikine6s(T, 'tol', 1e-3, 'pinv'); % 伪逆法求解
end

% 动画展示



% figure;
% robot.plot(q0, 'nobase', 'noshadow', 'nowrist', 'delay', 0);
% hold on;
% h_ideal1 = plot3(x, y, z, 'r--', 'LineWidth', 2); % 理想轨迹
% h_actual_nurbs = plot3(0,0,0, 'b-', 'LineWidth', 1.5); % 实际轨迹
% for k = 1:length(q_nurbs)
%     robot.animate(q_nurbs(k,:)); % 更新机械臂姿态
%     
%     % 获取末端实际位置
%     T_current = robot.fkine(q_nurbs(k,:));
%     current_pos = T_current.t;
%     
%     % 更新轨迹线
%     set(h_actual_nurbs, 'XData', [get(h_actual_nurbs, 'XData'), current_pos(1)], ...
%                   'YData', [get(h_actual_nurbs, 'YData'), current_pos(2)], ...
%                   'ZData', [get(h_actual_nurbs, 'ZData'), current_pos(3)]);
%     drawnow;
%     pause(0.02); 
% end
% 
% figure;
% robot.plot(q0, 'nobase', 'noshadow', 'nowrist', 'delay', 0);
% hold on;
% h_ideal2 = plot3(x, y, z, 'r--', 'LineWidth', 2); % 理想轨迹
% h_actual_cubic = plot3(0,0,0, 'g-', 'LineWidth', 1.5); % 实际轨迹
% for k = 1:length(q_cubic)
%     robot.animate(q_cubic(k,:)); % 更新机械臂姿态
%     
%     % 获取末端实际位置
%     T_current = robot.fkine(q_cubic(k,:));
%     current_pos = T_current.t;
%     
%     % 更新轨迹线
%     set(h_actual_cubic, 'XData', [get(h_actual_cubic, 'XData'), current_pos(1)], ...
%                   'YData', [get(h_actual_cubic, 'YData'), current_pos(2)], ...
%                   'ZData', [get(h_actual_cubic, 'ZData'), current_pos(3)]);
%     drawnow;
%     pause(0.02); 
% end
% 
% figure;
% robot.plot(q0, 'nobase', 'noshadow', 'nowrist', 'delay', 0);
% hold on;
% h_ideal3 = plot3(x, y, z, 'r--', 'LineWidth', 2); % 理想轨迹
% h_actual_quintic = plot3(0,0,0, 'y-', 'LineWidth', 1.5); % 实际轨迹
% for k = 1:length(q_quintic)
%     robot.animate(q_quintic(k,:)); % 更新机械臂姿态
%     
%     % 获取末端实际位置
%     T_current = robot.fkine(q_quintic(k,:));
%     current_pos = T_current.t;
%     
%     % 更新轨迹线
%     set(h_actual_quintic, 'XData', [get(h_actual_quintic, 'XData'), current_pos(1)], ...
%                   'YData', [get(h_actual_quintic, 'YData'), current_pos(2)], ...
%                   'ZData', [get(h_actual_quintic, 'ZData'), current_pos(3)]);
%     drawnow;
%     pause(0.02); 
% end


% 修正NURBS逆解循环范围
q_nurbs = zeros(length(x), 6);
for i = 1:length(x)
    T = transl(x(i), y(i), z(i)) * eul2tr([0, pi/2, 0]);
    q_nurbs(i,:) = robot.ikine6s(T, 'tol', 1e-3, 'pinv');
end

% 计算各轨迹的末端位置、速度、加速度
% 三次多项式轨迹
t_cubic = linspace(0, 10, 200)';
pos_cubic = zeros(length(t_cubic), 3);
for k = 1:length(t_cubic)
    T = robot.fkine(q_cubic(k,:));
    pos_cubic(k,:) = T.t';
end
% 计算速度和加速度
vel_cubic = gradient(pos_cubic) ./ gradient(t_cubic);
vel_cubic_mag = sqrt(sum(vel_cubic.^2, 2));
acc_cubic = gradient(vel_cubic) ./ gradient(t_cubic);
acc_cubic_mag = sqrt(sum(acc_cubic.^2, 2));

% 五次多项式轨迹
t_quintic = linspace(0, 10, size(q_quintic,1))'; % 假设总时间10秒
pos_quintic = zeros(size(q_quintic,1), 3);
for k = 1:size(q_quintic,1)
    T = robot.fkine(q_quintic(k,:));
    pos_quintic(k,:) = T.t';
end

% 修正后的NURBS逆解计算（包含无效点处理）
q_nurbs = zeros(length(x), 6);
for i = 1:length(x)
    T = transl(x(i), y(i), z(i)) * eul2tr([0, pi/2, 0]);
    q_sol = robot.ikine6s(T, 'tol', 1e-3, 'pinv');
    if any(isnan(q_sol))
        q_nurbs(i,:) = []; % 直接跳过无效点
    else
        q_nurbs(i,:) = q_sol;
    end
end

% 同步轨迹数据（删除无效点对应的x,y,z）
valid_idx = ~any(isnan(q_nurbs), 2);
q_nurbs = q_nurbs(valid_idx, :);
x = x(valid_idx);
y = y(valid_idx);
z = z(valid_idx);
% 计算末端位置
pos_nurbs = zeros(length(q_nurbs), 3);
for k = 1:length(q_nurbs)
    T = robot.fkine(q_nurbs(k,:));
    pos_nurbs(k,:) = T.t';
end

t_nurbs = linspace(0, 10, length(q_nurbs))'; % 总时间10秒，与三次/五次轨迹对齐
% 计算速度（梯度法，假设时间均匀）
vel_nurbs = gradient(pos_nurbs) ./ gradient(t_nurbs);
vel_mag_nurbs = sqrt(sum(vel_nurbs.^2, 2)); % 速度大小

% 计算加速度（二次梯度）
acc_nurbs = gradient(vel_nurbs) ./ gradient(t_nurbs);
acc_mag_nurbs = sqrt(sum(acc_nurbs.^2, 2)); % 加速度大小
% 计算速度和加速度
vel_quintic = gradient(pos_quintic) ./ gradient(t_quintic);
vel_quintic_mag = sqrt(sum(vel_quintic.^2, 2));
acc_quintic = gradient(vel_quintic) ./ gradient(t_quintic);
acc_quintic_mag = sqrt(sum(acc_quintic.^2, 2));

vel_nurbs_mag = zeros(length(x), 6);
for i = 1:length(x) % 循环范围修正为1到length(x)
    T = transl(x(i), y(i), z(i)) * eul2tr([0, pi/2, 0]);
   vel_nurbs_mag(i,:) = robot.ikine6s(T, 'tol', 1e-3, 'pinv');
end


% 线速度对比
figure;
hold on;
plot(t_cubic, vel_cubic_mag, 'b', 'LineWidth', 1.5, 'DisplayName', 'Cubic');
plot(t_quintic, vel_quintic_mag, 'g', 'LineWidth', 1.5, 'DisplayName', 'Quintic');
plot(t_nurbs, vel_mag_nurbs, 'r', 'LineWidth', 1.5, 'DisplayName', 'NURBS');
xlabel('Time (s)');
ylabel('Linear Velocity (m/s)');
legend;
title('Trajectory Velocity Comparison');
grid on;

% 加速度对比
figure;
hold on;
plot(t_cubic, acc_cubic_mag, 'b', 'LineWidth', 1.5, 'DisplayName', 'Cubic');
plot(t_quintic, acc_quintic_mag, 'g', 'LineWidth', 1.5, 'DisplayName', 'Quintic');
plot(t_nurbs, acc_mag_nurbs, 'r', 'LineWidth', 1.5, 'DisplayName', 'NURBS');
xlabel('Time (s)');
ylabel('Linear Acceleration (m/s²)');
legend;
title('Trajectory Acceleration Comparison');
grid on;
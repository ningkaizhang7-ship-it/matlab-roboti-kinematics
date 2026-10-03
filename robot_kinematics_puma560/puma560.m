% 加载PUMA560模型
mdl_puma560;
robot = p560;

% 设置采样点数
N = 10000;
q = zeros(N, 6);

% 生成随机关节角度
for i = 1:6
    q(:,i) = (robot.qlim(i,2) - robot.qlim(i,1)) * rand(N, 1) + robot.qlim(i,1);
end

% 计算末端位置
points = zeros(N, 3);
for i = 1:N
    T = robot.fkine(q(i,:));
    points(i,:) = T.t';
end

% 绘制工作空间
figure;
scatter3(points(:,1), points(:,2), points(:,3), 1, '.');
title('Monte Carlo 工作空间');
xlabel('X'); ylabel('Y'); zlabel('Z');


clc; clear; close all;
%% 1. 定义机械臂模型（改进D-H参数）
% 使用Peter Corke Robotics Toolbox创建PUMA560模型
mdl_puma560; % 加载工具箱内置模型
p560 = p560.nofriction(); % 获取无摩擦模型
%% 2. 设置关节参数
q_limits = p560.qlim; % 获取默认关节限位
% 手动调整限位（可选）
q_limits(1,:) = deg2rad([-160 160]);
q_limits(2,:) = deg2rad([-110 110]);
q_limits(3,:) = deg2rad([-135 135]);
%% 3. 数值法计算工作空间
N = 30; % 每个关节采样点数（建议30-50）
step = (q_limits(:,2)-q_limits(:,1))/N;
% 预分配内存
total_points = N^3;
points = zeros(total_points,3);
idx = 1;
% 主计算循环（仅计算前3个关节，后3关节设为0）
tic;
for q1 = q_limits(1,1):step(1):q_limits(1,2)
for q2 = q_limits(2,1):step(2):q_limits(2,2)
for q3 = q_limits(3,1):step(3):q_limits(3,2)
q = [q1 q2 q3 0 0 0];
T = p560.fkine(q); % 正运动学计算
points(idx,:) = T.t'; % 记录末端位置
idx = idx + 1;
end
end
end
toc;
% 点云图
subplot(1,2,1);
scatter3(points(:,1),points(:,2),points(:,3),3,points(:,3));
title('数值法计算工作空间');
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
axis equal tight;
grid on; view(3); rotate3d on;
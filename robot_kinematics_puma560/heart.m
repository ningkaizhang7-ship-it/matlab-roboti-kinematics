% theta = linspace(0, 2*pi, 1000);  % 生成0到2π的极角θ
% a = 1;                             % 控制心形大小的参数
% r = a * (1 - cos(theta));          % 极坐标方程
% 
% % 绘制极坐标图
% polarplot(theta, r, 'r-', 'LineWidth', 2);
% title('极坐标心形曲线 (r = 1 - cosθ)');
% grid on;

theta = linspace(0, 2*pi, 1000);  % 参数θ范围
x = 16 * sin(theta).^3;            % x坐标方程
y = 13*cos(theta) - 5*cos(2*theta) - 2*cos(3*theta) - cos(4*theta); % y坐标方程

plot(x, y, 'r-', 'LineWidth', 2);
title('笛卡尔参数方程心形曲线');
axis equal;                        % 保持坐标轴比例一致
grid on;
fill(x, y, 'r');                   % 填充颜色（可选）
xlim([-20 20]);
ylim([-15 15]);
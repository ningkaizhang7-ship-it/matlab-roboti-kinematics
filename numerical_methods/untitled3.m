% 定义函数
a = abs(randn(1));
f = @(x) x - a * tan(x);

% 设置x的范围，避开不连续点
x_min = 0;
x_max = pi/2 - 0.1; % 留出一点空间，避免接近pi/2

% 绘制函数图像
fplot(f, [x_min, x_max]);

% 设置图像的标题和坐标轴标签
title('Function Plot of x - a*tan(x) = 0');
xlabel('x');
ylabel('f(x)');

% 添加网格线以便更好地观察图像
grid on;

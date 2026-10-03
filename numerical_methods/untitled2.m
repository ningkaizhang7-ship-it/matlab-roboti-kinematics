% 定义函数

f = @(x) x - 0.7172 * tan(x);

% 设置x的范围，避开不连续点
x_min = 0;
x_max = 4*pi; 

% 绘制函数图像
fplot(f, [x_min, x_max]);

% 设置图像的标题和坐标轴标签
title('Function Plot of x - a*tan(x) = 0');
xlabel('x');
ylabel('f(x)');

% 添加网格线以便更好地观察图像
grid on;


% 定义区间和容许误差
left = 0; % 从0开始
right = pi/2; % 最小正根不会超过pi/2
tolerance = 0.01; % 保留两位小数

% 调用二分法函数
[root, iterations] = bisection_method(f, left, right, tolerance, 100);

% 显示结果
fprintf('The smallest positive root is: %.2f\n', root);
fprintf('Found after %d iterations.\n', iterations);
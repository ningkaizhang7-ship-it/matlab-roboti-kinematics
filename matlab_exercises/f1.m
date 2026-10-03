
f = @(x) 3*x.^2;
a = 0;
b = 2;

n = 100;

dx = (b - a) / n;

% 用矩形法求解数值积分
sum1 = 0;
for i = 1:n
    x_i = a + (i - 1) * dx;
    sum1 = sum1 + f(x_i);
end
I1 = dx * sum1;

% 用梯形法求解数值积分
sum2 = 0;
for i = 1:n
    x_i = a + (i - 1) * dx;
    x_i1 = a + i * dx;
    sum2 = sum2 + (f(x_i) + f(x_i1)) / 2;
end
I2 = dx * sum2;

% 对比两种方法的结果
fprintf('矩形法的结果是：%.4f\n', I1);
fprintf('梯形法的结果是：%.4f\n', I2);
fprintf('两种方法的误差是：%.4f\n', abs(I1 - I2));

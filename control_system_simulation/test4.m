K = 50;
num_g = K*conv([0.05, 4], [0.02, 0.6, 25]);
den_g = conv(conv([1, 0], [300, 3, 0]), [400, 0, 5, 60]);
G = tf(num_g, den_g);
% 计算闭环传递函数
Phi = feedback(G, 1);
disp(Phi)
step_info = stepinfo(Phi);
disp(step_info);
[A, B, C, D] = tf2ss(num_g, den_g);
disp('状态空间矩阵A:');
disp(A);
disp('状态空间矩阵B:');
disp(B);
disp('状态空间矩阵C:');
disp(C);
disp('状态空间矩阵D:');
disp(D);

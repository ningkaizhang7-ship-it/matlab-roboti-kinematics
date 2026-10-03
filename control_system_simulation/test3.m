syms y(t)
eqn = 4*diff(y,t,2) - 3*diff(y,t) + 2*y == -t;
ySol = dsolve(eqn);
disp('通解为：');
disp(ySol);

syms y(t)
eqn = 4*diff(y,t,2) - 3*diff(y,t) + 2*y == -t;
cond1 = y(0) == 1;
cond2 = subs(diff(y,t),t,0)== 0;
conds = [cond1, cond2];
ySol = dsolve(eqn, conds);
disp('特解为：');
disp(ySol);


t = 0:0.1:10;
y_val = double(subs(ySol, t));
plot(t, y_val);
xlabel('t');
ylabel('y(t)');
title('微分方程特解在 t \in [0, 10] 区间的解曲线');
grid on;

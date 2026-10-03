function [root, iter] = bisection_method(f, a, b, tol, max_iter)
    % f: 目标函数
    % a, b: 区间的初始端点
    % tol: 容许误差
    % max_iter: 最大迭代次数
    % root: 方程的根
    % iter: 迭代次数

    fa = f(a);
    fb = f(b);
    if fa * fb > 0
        error('f(a) and f(b) do not have opposite signs');
    end

    for iter = 1:max_iter
        root = (a + b)/2;
        fr = f(root);
        fprintf('Iteration %d: x = %.4f, f(x) = %.4f\n', iter, root, fr);
        if abs(fr) < tol
            break;
        end
        if fa * fr < 0
            b = root;
            fb = fr;
        else
            a = root;
            fa = fr;
        end
        if b - a < tol
            break;
        end
    end
end


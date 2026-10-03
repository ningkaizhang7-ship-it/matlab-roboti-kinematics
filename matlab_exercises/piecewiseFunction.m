function p = piecewiseFunction(x1, x2)
    if x1 + x2 > 1
        p = 0.5457 * exp(-0.75*x2 -3.75*x1 -1.5*x1);
    elseif x1 + x2 >= -1 && x1 + x2 <= 1
        p = 0.7575 * exp(-x2^2 -6*x2^4);
    else
        p = 0.5457 * exp(-0.75*x2 -3.75*x1 + 15*x4);
    end
end
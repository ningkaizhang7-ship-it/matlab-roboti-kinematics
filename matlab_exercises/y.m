
function [y1,y2] = y (t)
    y1 = exp (-t/3); 
    y2 = y1 .* sin (3*t); 
end



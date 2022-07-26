function y = sat_func(x, limit)
max_l = limit;
min_l = -limit;

s = 2;

for i= 1:s
    if x(i) > max_l
        x(i) = max_l;
    elseif x(i) < min_l 
        x(i) = min_l;
    end

end

y = x;


end
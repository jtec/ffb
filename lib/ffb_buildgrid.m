function grid = ffb_buildgrid(lowerB, upperB, n)
%#codegen
grid = zeros(2, n(1)*n(2));
steps = zeros(2,1);
for k=1:2
    steps(k) = (upperB(k) - lowerB(k))/(n(k)-1);
end

k = 1;
one = lowerB(1);
two = lowerB(2);
for i1 = 1:n(1)
    for i2 = 1:n(2)
        grid(:, k) = [one; two];
        k = k+1;
        two = two + steps(2);
    end
    two = lowerB(2);
    one = one + steps(1);
end
end


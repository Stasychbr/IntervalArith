C = kinterval([1, 0.5;0.8, 3], [4, 0.7;1.2, 5]);
d = kinterval([-1;-3],[1;3]);
D = diag(diag(C));
E = innerminus(C, D);

tol = [-0.3316 0 0.1514 0.25 0.3316 0 -0.1514 -0.25 -0.3316;0.6530 0.6 0.5037 0 -0.6531 -0.6 -0.5637 0 0.6530];

n = 10;
x = kinterval([-1; -1], [1; 1]);
figure
plot([inf(x(1)), inf(x(1)), sup(x(1)), sup(x(1)), inf(x(1))], [inf(x(2)), sup(x(2)), sup(x(2)), inf(x(2)), inf(x(2))])
hold on
plot(mid(x(1)), mid(x(2)))
hold on
x_set = [x];
D_inv = diag(inv(diag(C)));
for i=1:n
    x = D_inv * innerminus(d, E * x);
    x_set = [x_set x];
    plot([inf(x(1)), inf(x(1)), sup(x(1)), sup(x(1)), inf(x(1))], [inf(x(2)), sup(x(2)), sup(x(2)), inf(x(2)), inf(x(2))])
    hold on
    plot(mid(x(1)), mid(x(2)))
    hold on
endfor
plot(tol(1,:), tol(2,:), '--')
figure
rads = rad(x_set);
plot(rads(1, :))
hold on
plot(rads(2, :))
hold on
legend('x1', 'x2')
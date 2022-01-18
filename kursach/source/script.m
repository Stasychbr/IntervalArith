clearvars
pkg load interval
addpath(genpath('./m'))
X1 = [0; 64; 128; 192; 256; 320; 384; 448];
X2 = [448; 384; 320; 256; 192; 128; 64; 0];
Y1 = [30;30;26;24;17;11;7; 0];
Y2 = [0;6;7;11;14;20;25;29];

figure
plot(X1, Y1, 'bo')
hold on 
grid on
plot(X2, Y2, 'ro')

figure
[b1, b0, w] = make_joint(X1, Y1, [-Inf 0], [0, Inf]);
b1_1 = b1;

plot(X1, X1 * b1 + b0, 'b--')
grid on
hold on
problem1 = ir_problem([X1.^0 X1], Y1, w);
ir_scatter(problem1, '.b');
hold on

[b1, b0, w] = make_joint(X2, Y2, [-Inf 0], [0, Inf]);
b1_2 = b1;
plot(X2, X2 * b1 + b0, 'r--')
hold on
problem2 = ir_problem([X2.^0 X2], Y2, w);
ir_scatter(problem2, '.r');
hold on

figure
X = [X1; X2];
Y = [Y1; Y2];
[b1, b0, w] = make_joint(X, Y, [-Inf 0], [0, Inf]);
plot(X, X * b1 + b0, 'm--')
hold on
grid on
problem3_1 = ir_problem([X1.^0 X1], Y1, w(1:size(X1, 1)));
problem3_2 = ir_problem([X2.^0 X2], Y2, w(size(X1, 1) + 1:end));
ir_scatter(problem3_1, '.b');
ir_scatter(problem3_2, '.r');
hold on

X_lb = X1 - (X1(2) - X1(1)) / 4;
X_ub = X1 + (X1(2) - X1(1)) / 4;
Y_lb = min(Y1, flip(Y2));
Y_ub = max(Y1, flip(Y2));
shade = [238 232 213] ./ 255;
red = [220 50 47] ./ 255;
[w, q, b0] = make_joint_boxes(X_lb, X_ub, Y_lb, Y_ub, b1);
w_2=w;
b0_2=b0;
Y_boxes = infsup(Y_lb - q, Y_ub + q);
X_boxes = infsup(X_lb - w, X_ub + w);
figure
plot(X_boxes, Y_boxes, shade, red)
hold on 
grid on
plot(X, X * b1 + b0, 'r--')
hold on

Y1_lb = Y1 - 1;
Y1_ub = Y1 + 1;
Y2_lb = flip(Y2) - 1;
Y2_ub = flip(Y2) + 1;
Y_k = kinterval(max(Y1_lb, Y2_lb), min(Y1_ub, Y2_ub));
X_k = kinterval([X_lb ones(size(X_lb, 1), 1)], [X_ub ones(size(X_ub, 1), 1)]);
b_set = [];
figure
for i=1:6
  cur_X = X_k(i:i+1, :);
  cur_Y = Y_k(i:i+1);
  [betta, ~] = subdiff(cur_X, cur_Y);
  plot([inf(betta(1)), inf(betta(1)), sup(betta(1)), sup(betta(1)), inf(betta(1))], [inf(betta(2)), sup(betta(2)), sup(betta(2)), inf(betta(2)), inf(betta(2))])
  hold on
  b_set = [b_set betta];
  display(betta)
  disp(i)
end
grid on
#b_i_lb = [min(inf(b_set(1)), inf(b_set(3))); min(inf(b_set(2)), inf(b_set(4)))];
#b_i_ub = [max(sup(b_set(1)), sup(b_set(3))); max(sup(b_set(2)), sup(b_set(4)))];
b_intersect = infsup([inf(b_set(3));sup(b_set(4))], [inf(b_set(end-1));sup(b_set(2))]);
plot(b_intersect(1), b_intersect(2), shade, red)
figure
X_boxes = infsup(X_lb, X_ub);
Y_kp = pro(Y_k);
plot(X_boxes, infsup(inf(Y_kp), sup(Y_kp)), shade, red)
hold on
plot(X1, Y1, 'bo')
hold on
plot(X2, Y2, 'ro')
hold on
full_X = [X1; X_ub(end)];
plot(full_X, full_X * mid(b_intersect(1)) + mid(b_intersect(2)), '--')
hold on
grid on
function [w, q, b_0] = make_joint_boxes(X_lb, X_ub, Y_lb, Y_ub, b_1)
  n = size(X_lb, 1);
  A_leq = [-b_1 * eye(n, n) ones(n, 1) -eye(n, n)];
  A_geq = [b_1 * eye(n, n) ones(n, 1) eye(n, n)];
  A = [A_leq; A_geq];
  b = [Y_ub - b_1 * X_lb; Y_lb - b_1 * X_ub];  
  c = ones(1, 2 * n + 1);
  c(n + 1) = 0;
  ct = char([ones(1, n) * "U" ones(1, n) * "L"]);
  vt = char([ones(1, 2 * n + 1) * "C"]);
  lb = zeros(1, 2 * n + 1);
  ub = ones(1, 2 * n + 1) * Inf;
  [solution, ~, ~, ~] = glpk(c, A, b, lb, ub, ct, vt);
  solution(abs(solution) < 10^-4) = 0;
  w = solution(1:n); # rad x
  b_0 = solution(n + 1);
  q = solution(n + 2 : end); # rad y
end
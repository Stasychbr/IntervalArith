function [b1, b0, w] = make_joint(X, Y, b_lb, b_ub)
  n = size(Y, 1);
  A_leq = [X ones(n, 1) -eye(n, n)];
  A_geq = [X ones(n, 1) eye(n, n)];
  A = [A_leq; A_geq];
  b = [Y; Y];  
  c = zeros(1, n + 2);
  c(3:end) = 1;
  ct = char([ones(1, n) * "U" ones(1, n) * "L"]);
  vt = char([ones(1, n + 2) * "C"]);
  lb = [b_lb zeros(1, n)];
  ub = [b_ub ones(1, n) * Inf];
  [solution, ~, ~, ~] = glpk(c, A, b, lb, ub, ct, vt);
  solution(abs(solution) < 10^-4) = 0;
  b1 = solution(1);
  b0 = solution(2);
  w = solution(3:end);
end
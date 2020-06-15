% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(sum(a))
%   subject to
%     abs(MAcol0(1)*viM2(1) + MAcol0(2)*viM1(1) + MAcol0(3)*a(1)) <= v_max
%     abs(MAcol1(1)*viM2(1) + MAcol1(2)*viM1(1) + MAcol1(3)*a(1)) <= v_max
%     abs(MAcol2(1)*viM2(1) + MAcol2(2)*viM1(1) + MAcol2(3)*a(1)) <= v_max
%     abs(MAcol0(1)*viM2(2) + MAcol0(2)*viM1(2) + MAcol0(3)*a(2)) <= v_max
%     abs(MAcol1(1)*viM2(2) + MAcol1(2)*viM1(2) + MAcol1(3)*a(2)) <= v_max
%     abs(MAcol2(1)*viM2(2) + MAcol2(2)*viM1(2) + MAcol2(3)*a(2)) <= v_max
%     abs(MAcol0(1)*viM2(3) + MAcol0(2)*viM1(3) + MAcol0(3)*a(3)) <= v_max
%     abs(MAcol1(1)*viM2(3) + MAcol1(2)*viM1(3) + MAcol1(3)*a(3)) <= v_max
%     abs(MAcol2(1)*viM2(3) + MAcol2(2)*viM1(3) + MAcol2(3)*a(3)) <= v_max
%     abs(MBcol0(1)*viM1(1) + MBcol0(2)*a(1) + MBcol0(3)*b(1)) <= v_max
%     abs(MBcol1(1)*viM1(1) + MBcol1(2)*a(1) + MBcol1(3)*b(1)) <= v_max
%     abs(MBcol2(1)*viM1(1) + MBcol2(2)*a(1) + MBcol2(3)*b(1)) <= v_max
%     abs(MBcol0(1)*viM1(2) + MBcol0(2)*a(2) + MBcol0(3)*b(2)) <= v_max
%     abs(MBcol1(1)*viM1(2) + MBcol1(2)*a(2) + MBcol1(3)*b(2)) <= v_max
%     abs(MBcol2(1)*viM1(2) + MBcol2(2)*a(2) + MBcol2(3)*b(2)) <= v_max
%     abs(MBcol0(1)*viM1(3) + MBcol0(2)*a(3) + MBcol0(3)*b(3)) <= v_max
%     abs(MBcol1(1)*viM1(3) + MBcol1(2)*a(3) + MBcol1(3)*b(3)) <= v_max
%     abs(MBcol2(1)*viM1(3) + MBcol2(2)*a(3) + MBcol2(3)*b(3)) <= v_max
%     abs(MCcol0(1)*a(1) + MCcol0(2)*b(1) + MCcol0(3)*c(1)) <= v_max
%     abs(MCcol1(1)*a(1) + MCcol1(2)*b(1) + MCcol1(3)*c(1)) <= v_max
%     abs(MCcol2(1)*a(1) + MCcol2(2)*b(1) + MCcol2(3)*c(1)) <= v_max
%     abs(MCcol0(1)*a(2) + MCcol0(2)*b(2) + MCcol0(3)*c(2)) <= v_max
%     abs(MCcol1(1)*a(2) + MCcol1(2)*b(2) + MCcol1(3)*c(2)) <= v_max
%     abs(MCcol2(1)*a(2) + MCcol2(2)*b(2) + MCcol2(3)*c(2)) <= v_max
%     abs(MCcol0(1)*a(3) + MCcol0(2)*b(3) + MCcol0(3)*c(3)) <= v_max
%     abs(MCcol1(1)*a(3) + MCcol1(2)*b(3) + MCcol1(3)*c(3)) <= v_max
%     abs(MCcol2(1)*a(3) + MCcol2(2)*b(3) + MCcol2(3)*c(3)) <= v_max
%
% with variables
%        a   3 x 1
%        b   3 x 1
%        c   3 x 1
%
% and parameters
%   MAcol0   3 x 1
%   MAcol1   3 x 1
%   MAcol2   3 x 1
%   MBcol0   3 x 1
%   MBcol1   3 x 1
%   MBcol2   3 x 1
%   MCcol0   3 x 1
%   MCcol1   3 x 1
%   MCcol2   3 x 1
%    v_max   1 x 1    positive
%     viM1   3 x 1
%     viM2   3 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.MAcol0, ..., params.viM2, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2020-06-13 17:15:50 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.

function [C,U] = bspline_deboor(n,t,P,U)
% Evaluate explicit B-spline at specified locations.
%
% Input arguments:
% n:
%    B-spline order (2 for linear, 3 for quadratic, etc.)
% t:
%    knot vector
% P:
%    control points, typically 2-by-m, 3-by-m or 4-by-m (for weights)
% u (optional):
%    values where the B-spline is to be evaluated, or a positive
%    integer to set the number of points to automatically allocate
%
% Output arguments:
% C:
%    points of the B-spline curve

% Copyright 2010 Levente Hunyadi

validateattributes(n, {'numeric'}, {'positive','integer','scalar'});
d = n-1;  % B-spline polynomial degree (1 for linear, 2 for quadratic, etc.)
validateattributes(t, {'numeric'}, {'real','vector'});
assert(all( t(2:end)-t(1:end-1) >= 0 ), 'bspline:deboor:InvalidArgumentValue', ...
    'Knot vector values should be nondecreasing.');
validateattributes(P, {'numeric'}, {'real','2d'});
nctrl = numel(t)-(d+1);
assert(size(P,2) == nctrl, 'bspline:deboor:DimensionMismatch', ...
    'Invalid number of control points, %d given, %d required.', size(P,2), nctrl);
if nargin < 4
    U = linspace(t(d+1), t(end-d), 10*size(P,2));  % allocate points uniformly
elseif isscalar(U) && U > 1
    validateattributes(U, {'numeric'}, {'positive','integer','scalar'});
    U = linspace(t(d+1), t(end-d), U);  % allocate points uniformly
else
    validateattributes(U, {'numeric'}, {'real','vector'});
    assert(all( U >= t(d+1) & U <= t(end-d) ), 'bspline:deboor:InvalidArgumentValue', ...
        'Value outside permitted knot vector value range.');
end

m = size(P,1);  % dimension of control points
t = t(:).';     % knot sequence
U = U(:);
S = sum(bsxfun(@eq, U, t), 2);  % multiplicity of u in t (0 <= s <= d+1)
I = bspline_deboor_interval(U,t);

Pk = zeros(m,d+1,d+1);
a = zeros(d+1,d+1);

C = zeros(size(P,1), numel(U));
for j = 1 : numel(U)
    u = U(j);
    s = S(j);
    ix = I(j);
    Pk(:) = 0;
    a(:) = 0;

    % identify d+1 relevant control points
    Pk(:, (ix-d):(ix-s), 1) = P(:, (ix-d):(ix-s));
    h = d - s;

    if h > 0
        % de Boor recursion formula
        for r = 1 : h
            q = ix-1;
            for i = (q-d+r) : (q-s);
                a(i+1,r+1) = (u-t(i+1)) / (t(i+d-r+1+1)-t(i+1));
                Pk(:,i+1,r+1) = (1-a(i+1,r+1)) * Pk(:,i,r) + a(i+1,r+1) * Pk(:,i+1,r);
            end
        end
        C(:,j) = Pk(:,ix-s,d-s+1);  % extract value from triangular computation scheme
    elseif ix == numel(t)  % last control point is a special case
        C(:,j) = P(:,end);
    else
        C(:,j) = P(:,ix-d);
    end
end

function ix = bspline_deboor_interval(u,t)
% Index of knot in knot sequence not less than the value of u.
% If knot has multiplicity greater than 1, the highest index is returned.

i = bsxfun(@ge, u, t) & bsxfun(@lt, u, [t(2:end) 2*t(end)]);  % indicator of knot interval in which u is
[row,col] = find(i);
[row,ind] = sort(row);  %#ok<ASGLU> % restore original order of data points
ix = col(ind);
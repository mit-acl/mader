function output=lagrangePoly(xp,yp,xx)
%Example (the most important):
%xp=[1 2 3]; lagrangePoly(xp) 
%outputs the matrix of polynomial coefficients [0.5 -2.5 3.0; -1.0 4.0 -3.0; 0.5 -1.5 1.0] providing
%Lagrange subpolynomials L1(x)=0.5*x^2-2.5*x+3, L2(x)=-x^2+4*x-3, L3(x)=0.5*x^2-1.5*x+1 corresponding to nodes xp 
%
%Other examples (provided only for demonstration, covered equivalently by default Matlab functions such as polyfit etc.):
%xp=[1 2 3]; yp=[3 5 6]; lagrangePoly(xp,yp)
%outputs the coefficients [-0.5 3.5 0] of the Lagrange interpolation polynomial L(x)=-0.5*x^2+3.5*x+0
%passing through interpolation points (xp,yp), this choice provides the same result as polyfit(xp,yp,numel(xp)-1)
%
%xp=[1 2 3]; yp=[3 5 6]; xx=[1 1.2 1.3 1.5]; lagrangePoly(xp,yp,xx)
%outputs values [3.00 3.48 3.705 4.125] of the Lagrange interpolation polynomial L(x) (passing through interpolation points (xp,yp)) 
%in nodes xx (different from xp)
%
%xp=[1 2 3]; yp=[3 5 6]; xx=linspace(0,4); plot(xx,lagrangePoly(xp,yp,xx),xp,yp,'o')
%displays the Lagrange interpolation polynomial L(x) and the interpolation points (xp,yp)
% 
%xp=linspace(-1,1,9); a=5; fp=1./(1+a*xp.^2); xx=linspace(-1,1); ff=1./(1+a*xx.^2); plot(xx,lagrangePoly(xp,fp,xx),xp,fp,'o',xx,ff)
%displays the famous Runge function and its Lagrange interpolation polynomial L(x) passing through interpolation points (xp,fp)

coeffs=zeros(numel(xp),numel(xp));
for i=1:numel(xp)
    polynom=1;
    denominator=1;
    for j=1:numel(xp)
        if j~=i
           polynom=conv(polynom,[1 -xp(j)]);
           denominator=denominator*(xp(i)-xp(j));
        end
    end
    coeffs(i,:)=polynom/denominator;
end

if nargin==1
   output=coeffs;
elseif nargin==2
    output=yp*coeffs;
else
    output=polyval(yp*coeffs,xx);
end
    
   


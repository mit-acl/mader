using TSSOS
using DynamicPolynomials
@polyvar x[1:2]
f=x[1]^4+x[2]^4-1.0*x[1]*x[2]
opt,sol,data=blockupop_first(f,x)

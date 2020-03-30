using DynamicPolynomials
using TSSOS
@polyvar x[1:2]
f=x[1]^4+x[2]^4-x[1]*x[2]
g_1=2-x[1]^2-2*x[2]^2
g_2=x[1]^2-x[2]^2-1
pop=[f,g_1,g_2]
d=2 # the order of Lasserre's hierarchy
opt,sol,data=blockcpop_first(pop,x,d,numeq=1,method="chordal")
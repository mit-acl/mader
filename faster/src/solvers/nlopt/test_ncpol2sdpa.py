from ncpol2sdpa import *
import numpy as np
from sympy import *

level = 6

deg=3;
num_el_B=deg-1;
num_el_R=((deg+1)/2)*((deg-1)/2)

B_var = generate_variables('B', num_el_B)
R_var = generate_variables('R', num_el_R)
t = generate_variables('t', 1)
t=t[0]
print(t)
B=B_var;
R=np.reshape(R_var, ((deg+1)/2, (deg-1)/2))

print(R)


np.matrix('1 2; 3 4')

W=[];
for i in range(0,((deg+1)/2)):
    pol=-B[i]*(t-1);
    for j in range(0,(deg-1)/2):
        pol=pol*((t-R[i,j])**2);
    W.append(pol)

for i in range(0,((deg+1)/2)):
    pol=-B[i]*(-t-1);
    for j in range(0,(deg-1)/2):
        pol=pol*((-t-R[i,j])**2);
    W.append(pol)

#Create the A matrix:
A=[];
for i in range(0,len(W)):
    tmp=poly(W[i],t)
    coeffic=tmp.coeffs()
    #print(tmp.coeffs())
    A.append(coeffic)
    #tmp=flip(coefficients(W(i),t))'
    #A=[A; tmp];


print("==================")

print(A)
#Convert to simpy matrix
A = Matrix(A)

print("========= Determinant: =========")

detA=A.det()

print(detA)

print("=========================")


obj=-detA;





coeff=np.sum(A, axis=0) #Sum of the columns
tmp=np.zeros(deg+1);
tmp[-1]=1.0;
print("tmp is ", tmp)
coeff_should_be=tmp

print("Sum of the coeff is")
# print(sum_coeff)

equalities= (coeff-coeff_should_be)


obj=-detA;
inequalities = flatten([B_var])+equalities.tolist() +list(-1.0*equalities);
# inequalities.append(equalities.tolist());
# inequalities.append(list(-1.0*equalities));

print("inequalities are", inequalities)

print(flatten([B_var, R_var]))

sdp=SdpRelaxation(flatten([B_var, R_var]))
sdp.get_relaxation(level, objective=obj, inequalities=inequalities, chordal_extension=True)
sdp.solve()
print(sdp.primal, sdp.dual)
println("Starting")
using TSSOS
using DynamicPolynomials
using LinearAlgebra
println("Starting")
deg=5
@polyvar t
@polyvar B[1:deg-1,1]
@polyvar R[1:convert(Int64,(deg+1)/2),1:convert(Int64,(deg-1)/2)]


W=zeros(0, 1)#[]

for i=1:convert(Int64,(deg+1)/2)
	println("S2")
    pol=-B[i]*(t-1);
	println("S3")
    for j=1:convert(Int64,(deg-1)/2)
        pol=pol*((t-R[i,j])^2);
    end
    println("S5")
    global W=vcat(W, pol)
#=    W=[W;pol];=#
    println("S6")
end

println("Size of W before is ", size(W))

W=vcat(W,subs(W, t=>-t))

println("Size of W after is ", size(W))

println("Size of R is ", size(R))
#W is a column vector now
sum_W=sum(W, dims=1) #Sum the columns

println("Size of sum_W is ", size(sum_W))

sum_W=sum_W[1,1]; #De-array it

function getCoeffs(pol,last_deg,variable)
	#@polyvar coeffi[1:1,0]
	coeffi=zeros(1, 0)
	for i=0:last_deg
		tmp=coefficient(pol,t^(last_deg-i),[variable])
		println(typeof(tmp))
		coeffi=[coeffi tmp]
		#println(coefficient(pol,t^(last_deg-i),[variable]))#powers_t[i],));
	end
	println("Done getting the coeffs")
	println(coeffi)
	return coeffi'
end
#=coeffi=getCoeffs(t^2+3*t+1,4, t)=#

coeffi=getCoeffs(sum_W,deg,t);

coeffi_should_be=[zeros(1, (deg)) 1]
coeffi_should_be=coeffi_should_be[1,:]

println("Coeff are")

println(coeffi)
println("=========")

println(coeffi_should_be)

add_dim(x::Array) = reshape(x, (size(x)...,1))

#@polyvar A[1:deg+1,0]
A=zeros(deg+1, 0)
for i=1:size(W, 1)
    tmp=getCoeffs(W[i,1],deg,t);#coefficients(W[i,1],T);
    global A=[A tmp]#vcat(A,tmp);
end

coeffi_should_be=add_dim(coeffi_should_be) #convert from (X,) to (X,1)  (column vector)

detA=-1.0*det(A)

println("THIS IS detA")
println(detA)

B_row=reshape(B,1,length(B))
R_row=reshape(R,1,length(R))

BR_row=[B_row R_row]


@polyvar BR[1:length(BR_row)]

for i=1:length(BR_row)
	 BR[i]=BR_row[1,i]  
end

pop=[detA B_row (coeffi-coeffi_should_be)']

println("THIS IS POP BEFORE")
println(pop)

d=6 # the order of Lasserre's hierarchy

pop=pop[1,:]
BR_row=BR_row[1,:]


pop=subs(pop, t=>0.0) #need to do this (although it has no effect because t doesn't appear), but if not I've

#=pop=subs(pop, t=>0.0)
BR_row=subs(BR_row, t=>0.0)
=#
println("THIS IS POP")
println(pop)

println("THIS IS BR_row")

println(BR_row)


println("Starting Optimization!")
opt,sol,data=blockcpop_first(pop,BR_row,d,numeq=(deg+1),solution=true,method="chordal",chor_alg="greedy") #,method="chordal"
println("Finishing")


println(sol)
println(opt)
println(data)


#=
println("Size of BR_row is ", size(BR_row))
println("Size of coeffi AFTER is ", size(coeffi))
println("Size of coeffi_should_be AFTER is ", size(coeffi_should_be))

println(detA)

println("Type of detA is ", typeof(detA))
println("Type of B[1,1] is ", typeof(B[1,1]))

println("======================================")

println(coeffi)

println("==")
println(coeffi_should_be)
println("==")


println((coeffi-coeffi_should_be)')

println("======================================")


println("Starting Optimization!")
println("TYPE OF POP")

println("TYPE OF pop")
println(typeof(pop))

println("TYPE OF BR_row")
println(typeof(BR_row))=#
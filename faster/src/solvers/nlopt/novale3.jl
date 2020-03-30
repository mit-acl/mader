using TSSOS
using DynamicPolynomials
using SparseArrays

@polyvar x0 x1 x2 x3
x = tuple(x0,x1,x2,x3)
f=include("standard_4_10_12.txt")
n=length(x)
mon=monomials(f)
coe=coefficients(f)
lm=length(mon)
supp=zeros(UInt8,n,lm)
for i=1:lm
    for j=1:n
        supp[j,i]=MultivariatePolynomials.degree(mon[i],x[j])
    end
end
d=Int(maxdegree(f)/2)

n=10
@polyvar x[1:10]
f=x[1]+1
for i=2:n
    global f+=100*(x[i]-x[i-1]^2)^2+(1-x[i])^2
end
f-=x[1]
for i=1:n
    for j=i+1:n
        global f+=x[i]^2*x[j]^2
    end
end

@time begin
opt,sol,data=blockupop_first(f,x,newton=0,method="block",QUIET=true)
end

@time begin
opt,sol,data=blockupop_higher!(data,method="block",QUIET=true)
end

sol=extract_solutions(n,0,[],d,[],0,opt,basis,blocks,cl,blocksize,Gram,method="block")

n=12
d=4
ms=MSession()
mat"x = sym('x',[1 $n]);
load E:\\Programs\\blocksos\\poly2_12_4_25.mat;
% poly=1+x(1)^4+x(2)^4+x(3)^4+x(1)*x(2)*x(3)+x(2);
% poly=1+x(1)^2*x(2)^2-x(1)*x(2);
% syms x1 x2 x3;
% poly=1+x1^3+x1^4-3*x1*x2-4*x1^2*x2+2*x2^2+x2^3+x1*x2^3+x2^4;
% poly=pop(1);
% for i=1:$n
%     poly=poly-0.1*pop(i+1);
% end
[coe, terms] = coeffs(poly,x);
lt=length(terms);
supp=zeros($n,lt);
for i=1:lt
    for j=1:$n
        supp(j,i)=feval(symengine,'degree',terms(i),x(j));
    end
end
coe=double(coe)"
coe=jarray(get_mvariable(ms,:coe))
supp=jarray(get_mvariable(ms,:supp))
supp=convert(Array{UInt8},supp)

@time begin
if sum(supp[:,end])!=0
    supp=[supp zeros(UInt8,n,1)]
    coe=[coe 0]
end
basis=newton_tbasis(n,d,supp)
supp1=zeros(UInt8,n,1)
for i=1:cl
    for j=1:blocksize[i]
        for r=j:blocksize[i]
            bi=basis[:,blocks[i][j]]+basis[:,blocks[i][r]]
            global supp1=[supp1 bi]
        end
    end
end
basis=get_basis(n,d)
basis=generate_basis!(n,supp,basis)
tsupp=[supp 2*basis]
tsupp=sortslices(tsupp,dims=2)
tsupp=unique(tsupp,dims=2)
basis=get_basis(n,d)
basis=generate_basis!(n,tsupp,basis)
#tsupp=[supp 2*basis]
#basis=get_basis(n,d)
#basis=generate_basis!(n,tsupp,basis)
blocks,cl,blocksize,ub,sizes=get_blocks(n,supp,basis,reduce=1)
blocks,cl,blocksize,ub,sizes=get_hblocks!(n,supp1,basis,ub,sizes,reduce=1)
basis,flag=reducebasis!(n,supp,basis,blocks,cl,blocksize)
if flag==1
blocks,cl,blocksize,ub,sizes=get_cliques(n,supp,basis,reduce=0)
end
@time begin
opt,supp1=blockupop(n,supp,coe,basis,blocks,cl,blocksize,QUIET=false)
end

@time begin
blocks,cl,blocksize,ub,sizes,status=get_hcliques!(n,supp1,basis,ub,sizes,reduce=0,dense=1)
opt,supp1=blockupop(n,supp,coe,basis,blocks,cl,blocksize)
end

f=x[1]^2-2*x[1]*x[2]+3*x[2]^2-2*x[1]^2*x[2]+2*x[1]^2*x[2]^2-2*x[2]*x[3]+6*x[3]^2+18*x[2]^2*x[3]-54*x[2]*x[3]^2+142*x[2]^2*x[3]^2

@polyvar x[1:3]
f=x[1]^4*x[2]^2+x[1]^2*x[2]^4+1-3x[1]^2*x[2]^2
opt,sol,data=blockupop_first(f,x,newton=0,method="chordal",chor_alg="greedy")
opt,sol,data=blockupop_higher!(data,method="chordal",chor_alg="greedy")

-------------------------------------------------------------
n=2
m=1
d=6
dg=[3]
ms=MSession()
mat"x = sym('x',[1 $n]);
% load E:\\Programs\\blocksos\\cpoly_6_4_10.mat;
% f=x(1)^10+x(2)^10-x(1)*x(2)-x(1);
% g=1-2*x(1)^2-x(2)^2;
f=(x(1)+1)^2+x(2)^2;
% g=1-sum(x.^2);
% g=1-x(1)^2;
% g=[1-x(1)^2,1-x(2)^2,1-x(3)^2,1-x(4)^2,1-x(5)^2,1-x(6)^2];
g=x(1)^3-x(2)^2;
pop=[f,g];
coe=cell(1,$m+1);
terms=cell(1,$m+1);
ssupp=cell(1,$m+1);
supp=[];
lt=zeros(1,$m+1);
for k=1:$m+1
    [coe{k}, terms{k}] = coeffs(pop(k),x);
    lt(k)=length(terms{k});
    ssupp{k}=zeros($n,lt(k));
    for i=1:lt(k)
        for j=1:$n
            ssupp{k}(j,i)=feval(symengine,'degree',terms{k}(i),x(j));
        end
    end
    supp=[supp ssupp{k}];
end
for k=1:$m+1
    coe{k}=double(coe{k});
end"
coe=jarray(get_mvariable(ms,:coe))
supp=jarray(get_mvariable(ms,:supp))
supp=convert(Array{UInt8},supp)
supp=unique(supp,dims=2)
ssupp=jarray(get_mvariable(ms,:ssupp))
for k=1:m+1
    ssupp[k]=convert(Array{UInt8},ssupp[k])
end
lt=jarray(get_mvariable(ms,:lt))
lt=convert(Array{UInt32},lt)

fbasis=get_basis(n,d)
gbasis=Array{Any}(undef,m)
for k=1:m
    gbasis[k]=get_basis(n,d-Int(ceil(dg[k]/2)))
end
fblocks,fcl,fblocksize,gblocks,gcl,gblocksize,ub,sizes,status=get_ccliques(n,m,supp,ssupp,lt,fbasis,gbasis)
tsupp=[ssupp[1] zeros(UInt8,n,1)]
for k=1:m
    for i=1:gcl[k]
        for j=1:gblocksize[k][i]
            for r=j:gblocksize[k][i]
                for s=1:lt[k+1]
                    bi=ssupp[k+1][:,s]+gbasis[k][:,gblocks[k][i][j]]+gbasis[k][:,gblocks[k][i][r]]
                    global tsupp=[tsupp bi]
                end
            end
        end
    end
end
fbasis=generate_basis!(n,tsupp,fbasis)
fbasis,flag=reducebasis!(n,tsupp,fbasis,fblocks,fcl,fblocksize)
fblocks,fcl,fblocksize,gblocks,gcl,gblocksize,ub,sizes,status=get_ccliques(n,m,supp,ssupp,lt,fbasis,gbasis,reduce=1)
fblocks,fcl,fblocksize,gblocks,gcl,gblocksize,ub,sizes,status=get_chcliques!(n,m,ssupp,lt,fbasis,gbasis,fsupp,ub,sizes,reduce=1)
opt,fsupp,gsupp=blockcpop(n,m,ssupp,coe,lt,fbasis,gbasis,fblocks,fcl,fblocksize,gblocks,gcl,gblocksize)

@polyvar x[1:2]
f=x[1]^4+x[2]^4-x[1]*x[2]+1
g=1-x[1]^2-2*x[2]^2
h=x[1]^2+x[2]^2-1
pop=[f,g,h]

@polyvar x[1:2]
f=x[1]^4*x[2]^2+x[1]^2*x[2]^4-3x[1]^2*x[2]^2+1
g=4-x[1]^2-x[2]^2
pop=[f,g]

n=10
@polyvar x[1:10]
f=x[n]*x[1]
for i=1:n-1
    global f+=x[i]*x[i+1]
end
pop=[f]
for i=1:n
    push!(pop,1-x[i]^2)
end
pop=[f,1-sum(x.^2)]

@time begin
opt,sol,data=blockcpop_first(pop,x,2,method="chordal",chor_alg="greedy",numeq=0,QUIET=false)
end

@time begin
opt,sol,data=blockcpop_higher!(data,method="chordal",chor_alg="greedy")
end

sol=extract_solutions(n,m,x,d,pop,numeq,opt,fbasis,fblocks,fcl,fblocksize,Gram,method="block")

----------------------------------------------
n=50
d=2
m=50
ms=MSession()
mat"x = sym('x',[1 $n]);
load E:\\Programs\\blocksos\\network2_50.mat;
% pop=[x(1)^4+x(2)^4-x(1)*x(2)+1,1-x(1)^4-x(2)^4];
coe=cell(1,$m+1);
terms=cell(1,$m+1);
ssupp=cell(1,$m+1);
supp=[];
lt=zeros(1,$m+1);
for k=1:$m+1
    [coe{k}, terms{k}] = coeffs(pop(k),x);
    lt(k)=length(terms{k});
    ssupp{k}=zeros($n,lt(k));
    for i=1:lt(k)
        for j=1:$n
            ssupp{k}(j,i)=feval(symengine,'degree',terms{k}(i),x(j));
        end
    end
    supp=[supp ssupp{k}];
end
for k=1:$m+1
    coe{k}=double(coe{k});
end"

@time begin
coe=jarray(get_mvariable(ms,:coe))
ssupp=jarray(get_mvariable(ms,:ssupp))
supp=jarray(get_mvariable(ms,:supp))
lt=jarray(get_mvariable(ms,:lt))
for k=1:m+1
    ssupp[k]=convert(Array{UInt8},ssupp[k])
end
supp=convert(Array{UInt8},supp)
lt=convert(Array{UInt16},lt)
supp=unique(supp,dims=2)
basis=get_basis(n,d)
blocks,cl,blocksize,ub,sizes,status=get_cliques(n,supp,basis)
opt,supp1=blockcsos(n,m,ssupp,coe,lt,basis,blocks,cl,blocksize)
end

-------------------------------------------------------------------
n=6
d=2
@polyvar x[1:n]
f=1+sum(x.^4)+x[1]*x[2]*x[3]+x[3]*x[4]*x[5]+x[3]*x[4]*x[6]+x[3]*x[5]*x[6]+x[4]*x[5]*x[6]

mon=monomials(f)
coe=coefficients(f)
lm=length(mon)
supp=zeros(UInt8,n,lm)
for i=1:lm
    for j=1:n
        supp[j,i]=MultivariatePolynomials.degree(mon[i],x[j])
    end
end
supp=sparse(supp)

cliques,cql,cliquesize=clique_decomp(n,supp)
mclique,lmc,blocks,cl,blocksize,ub,sizes,basis=get_blocks_mix(d,supp,cliques,cql,cliquesize,ts=4,method="block")
blocks,cl,blocksize,ub,sizes=get_hblocks_mix(basis,mclique,lmc,cliquesize,blocks,cl,blocksize,ub,sizes,method="block")
objv=blockupop_mix(n,d,supp,coe,cliques,cql,cliquesize,mclique,lmc,blocks,cl,blocksize,ts=4,QUIET=true)

-------------------------------------------------------------------
n=length(x)
m=length(pop)-1
numeq=0
@polyvar x[1:n]
f=1+sum(x.^4)+x[1]*x[2]*x[3]+x[3]*x[4]*x[5]+x[4]*x[5]*x[6]
g1=1-sum(x[1:3].^2)
g2=1-sum(x[3:6].^2)
g3=x[5]
pop=[f,g1,g2,g3]
coe=Array{Vector{Float64}}(undef, m+1)
supp=Array{SparseMatrixCSC}(undef, m+1)
for k=1:m+1
    mon=monomials(pop[k])
    coe[k]=coefficients(pop[k])
    lt=length(mon)
    ssupp=zeros(UInt8,n,lt)
    for i=1:lt
        for j=1:n
            ssupp[j,i]=MultivariatePolynomials.degree(mon[i],x[j])
        end
    end
    supp[k]=sparse(ssupp)
end

rlorder=ones(Int, m+1)
rlorder[1]=2
cliques,cql,cliquesize=clique_cdecomp(n,m,supp,rlorder,alg="amd")
ts=5

@time begin
mclique,I,ncc,lmc,blocks,cl,blocksize,ub,sizes,ssupp,lt,fbasis,gbasis=get_cblocks_mix(rlorder,m,supp,cliques,cql,cliquesize,ts=ts,method="chordal",chor_alg="amd")
objv,supp1=blockcpop_mix(n,m,rlorder,supp,coe,cliques,cql,cliquesize,mclique,I,ncc,lmc,blocks,cl,blocksize,numeq=numeq,ts=ts,QUIET=false,solve=true)
end

@time begin
blocks,cl,blocksize,ub,sizes,status=get_chblocks_mix!(I,supp1,ssupp,lt,fbasis,gbasis,mclique,lmc,cliques,cliquesize,blocks,cl,blocksize,ub,sizes,method="chordal",chor_alg="amd")
objv,supp1=blockcpop_mix(n,m,rlorder,supp,coe,cliques,cql,cliquesize,mclique,I,ncc,lmc,blocks,cl,blocksize,numeq=numeq,ts=ts,QUIET=false,solve=true)
end
-------------------------------------------------------------------
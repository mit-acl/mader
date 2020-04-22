clc; close all;clear;
set(0,'DefaultFigureWindowStyle','docked');

global sol1 sol2 sol3 sol5 T1 T2 T3 T5
global t
syms t real
T1=[t 1]';
T2=[t*t t 1]';
T3=[t*t*t t*t t 1]';
T4=[t*t*t*t t*t*t t*t t 1]';
T5=[t^5 T4']';
T6=[t^6 T5']';
T7=[t^7 T6']';

interv=[-1,1];

sol1=load('solutionDeg1.mat');
sol2=load('solutionDeg2.mat');
sol3=load('solutionDeg3.mat');
% sol4=load('solutionDeg4.mat');
sol5=load('solutionDeg5.mat');
sol7=load('solutionDeg7.mat');

sol1.A=double(vpa(sol1.A,4));
sol2.A=double(vpa(sol2.A,4));
sol3.A=double(vpa(sol3.A,4));
sol5.A=double(vpa(sol5.A,4));
sol7.A=double(vpa(sol7.A,4));



%%
Abz=computeMatrixForBezier(3)
%%
% 
% sol1=transformStructureTo01(sol1);
% sol2=transformStructureTo01(sol2);
% sol3=transformStructureTo01(sol3);
% sol5=transformStructureTo01(sol5);
% sol7=transformStructureTo01(sol7);

sol1.Ai=inv(sol1.A);
sol2.Ai=inv(sol2.A);
sol3.Ai=inv(sol3.A);
sol5.Ai=inv(sol5.A);
sol7.Ai=inv(sol7.A);

%Note that these are defined for [0 1]
sol1.Abz=computeMatrixForBezier(1);
sol2.Abz=computeMatrixForBezier(2);
sol3.Abz=computeMatrixForBezier(3);
sol5.Abz=computeMatrixForBezier(5);
sol7.Abz=computeMatrixForBezier(7);

%%Swap stuff
tmp1=sol2.A(1,:);
tmp3=sol2.A(3,:);
sol2.A(1,:)=tmp3;
sol2.A(3,:)=tmp1;

tmp1=sol3.A(1,:);
tmp2=sol3.A(2,:);
tmp3=sol3.A(3,:);
tmp4=sol3.A(4,:);
sol3.A=[tmp2; tmp3; tmp1; tmp4];

tmp1=sol5.A(1,:);
tmp2=sol5.A(2,:);
tmp3=sol5.A(3,:);
tmp4=sol5.A(4,:);
tmp5=sol5.A(5,:);
tmp6=sol5.A(6,:);
sol5.A=[tmp3; tmp4; tmp2; tmp5; tmp1; tmp6];
% sol5.A=[tmp4; tmp1; tmp5; tmp3; tmp2; tmp6];

tmp1=sol7.A(1,:);
tmp2=sol7.A(2,:);
tmp3=sol7.A(3,:);
tmp4=sol7.A(4,:);
tmp5=sol7.A(5,:);
tmp6=sol7.A(6,:);
tmp7=sol7.A(7,:);
tmp8=sol7.A(8,:);
sol7.A=[tmp1;tmp7;tmp4;tmp6;tmp2;tmp8;tmp3;tmp5;];

%Compute now the matrix for the Lagrange polynomials, which are very
%similar:
sol1.Al=lagrangePoly(linspace(-1,1,2));
sol2.Al=lagrangePoly(linspace(-1,1,3));
sol3.Al=lagrangePoly(linspace(-1,1,4));
% sol4.Al=lagrangePoly(linspace(-1,1,5));
sol5.Al=lagrangePoly(linspace(-1,1,6));

%%

% 
% sol3.rootsA= sol3.rootsA(1:end/2, :);
% sol5.rootsA= sol5.rootsA(1:end/2, :);

% sol2.A=sol2.A(1:2,:);
% sol3.A=sol3.A(1:2,:);

scatter(3*ones(size(sol3.rootsA(:))), sol3.rootsA(:)); hold on
% scatter(4*ones(size(sol4.rootsA(:))), sol4.rootsA(:))
scatter(5*ones(size(sol5.rootsA(:))), sol5.rootsA(:))

figure
sol3.rootsTA=transformTo01(sol3.rootsA);
sol5.rootsTA=transformTo01(sol5.rootsA);
scatter(3*ones(size(sol3.rootsTA(:))), sol3.rootsTA(:)); hold on
% scatter(4*ones(size(sol4.rootsA(:))), sol4.rootsA(:))
scatter(5*ones(size(sol5.rootsTA(:))), sol5.rootsTA(:))


S=load('solutionDeg3.mat')
A=S.A;

%odd columns of A
oddcol=A(:,1:2:end);

%even columns 
evencol=A(:,2:2:end);

A_reorganized=[oddcol,evencol];

nrow=size(A,1);
ncol=size(A,2);

AA=A_reorganized(1:nrow/2,1:ncol/2);

BB=A_reorganized(1:nrow/2,ncol/2+1:end);

det(A)

det(2*AA)*det(BB)


det(AA)/det(BB)

disp("=============")


T=[t*t t 1]';


syms a b c d real



polys1=vpa(sol1.A*T1,4);
polys2=vpa(sol2.A*T2,4);
polys3=vpa(sol3.A*T3,4);
%%
clc
figure; hold on;
syms a b c d real
comb=(a*t+b)*polys1(1) + (c*t+d)*polys2(1);
coeff_comb=vpa(coeffs(comb,t,'All'),4);
sol=solve(coeff_comb(1:4)==sol3.A(1,1:4))

a=sol.a; b=sol.b; c=sol.c; d=sol.d;
comb=(a*t+b)*polys1(1) + (c*t+d)*polys2(1);

fplot(comb,[0 1])
% coeff_comb=vpa(coeffs(comb,t,'All'),4);
%%

% disp("=======================================")
% comb=(a*t +b)*(sol2.A(1,:)*T2) + (c*t+d)*(sol2.A(2,:)*T2 );
% coeff_comb=vpa(coeffs(comb,t,'All'),4)
% sol=solve(coeff_comb==sol3.A(1,:));


syms tn real %t_new
tn=t*0.577;
Tn2=[tn*tn tn 1]';
Tn3=[tn*tn*tn     tn*tn     tn     1     ]';
Tn4=[tn*tn*tn*tn  tn*tn*tn  tn*tn  tn 1  ]';
figure
fplot(2*(sol2.A*Tn2-[0;0.5;0]),interv);

disp("=======================================")
%%
comb=combine2pol(polys1(1),polys2(1),a,b,c,d);
coeff_comb=vpa(coeffs(comb,t,'All'),4);
sol=solve(coeff_comb==sol3.A(1,:));


disp("combining first")
comb=combine2pol(polys1(1),polys2(1),sol.a,sol.b,sol.c,sol.d);
coeff_comb=vpa(coeffs(comb,t,'All'),4)
figure; hold on;
fplot(coeff_comb*T3,interv);

disp("combining more")
comb=combine2pol(polys1(1),polys2(2),sol.a,sol.b,sol.c,sol.d);
coeff_comb=vpa(coeffs(comb,t,'All'),4)
fplot(coeff_comb*T3,interv);

%%
clc
Q1=sym('Q1_%d%d',[4,2],'real');
Q2=sym('Q2_%d%d',[2,4],'real');
Q=[Q1(:); Q2(:)];

tmp=Q1*sol1.A*Q2;
s=solve(tmp(1:2,1:2)==sol3.A(1:2,1:2))
%%
figure; hold on;

% sol1.AT=sol1.A';
% sol2.AT=sol2.A';
% sol3.AT=sol3.A';
% sol5.AT=sol5.A';
% sol7.AT=sol7.A';

sol1.AT=sol1.A';
sol2.AT=sol2.A';
sol3.AT=sol3.A';
sol5.AT=sol5.A(1:3,:);
sol7.AT=sol7.A(1:4,:);

tmp=[zeros(1,3); [0 normalize(sol1.AT(1,:))]];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','red','LineStyle','--')
tmp=[zeros(1,3); [0 normalize(sol1.AT(2,:))]];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','red','LineStyle','--')

% tmp=[zeros(1,2); sol1.AT(1,:)];
% line(tmp(:,1),tmp(:,2),'Color','red','LineStyle','--' )
% tmp=[zeros(1,2); sol1.AT(2,:)];
% line(tmp(:,1),tmp(:,2),'Color','red','LineStyle','--' )

tmp=[zeros(1,3); normalize(sol2.AT(1,:))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','black','LineWidth',2 )
tmp=[zeros(1,3); normalize(sol2.AT(2,:))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','black','LineWidth',2 )
tmp=[zeros(1,3); normalize(sol2.AT(3,:))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','black','LineWidth',2 )

xlabel('x'); ylabel('y'); zlabel('z');

% 1:3
% end-2:end 

tmp=[zeros(1,3); normalize(sol3.AT(1,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','green','LineStyle','-','LineWidth',2 )
tmp=[zeros(1,3); normalize(sol3.AT(2,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','green','LineStyle','-')
tmp=[zeros(1,3); normalize(sol3.AT(3,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','green','LineStyle','-','LineWidth',2  )
tmp=[zeros(1,3); normalize(sol3.AT(4,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','green','LineStyle','-' )

tmp=[zeros(1,3); normalize(sol5.AT(1,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','blue','LineStyle','--' )
tmp=[zeros(1,3); normalize(sol5.AT(2,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','blue','LineStyle','--' )
tmp=[zeros(1,3); normalize(sol5.AT(3,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','blue','LineStyle','--' )
% % tmp=[zeros(1,3); sol5.AT(4,end-2:end)];
% % line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','blue','LineStyle','--' )
% % tmp=[zeros(1,3); sol5.AT(5,end-2:end)];
% % line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','blue','LineStyle','--' )
% % tmp=[zeros(1,3); sol5.AT(6,end-2:end)];
% % line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','blue','LineStyle','--' )
% 
tmp=[zeros(1,3); normalize(sol7.AT(1,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','magenta','LineStyle','-' )
tmp=[zeros(1,3); normalize(sol7.AT(2,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','magenta','LineStyle','-' )
tmp=[zeros(1,3); normalize(sol7.AT(3,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','magenta','LineStyle','-' )
tmp=[zeros(1,3); normalize(sol7.AT(4,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','magenta','LineStyle','-' )

% coord=[sol2.AT(1,:); sol2.AT(2,:); sol3.AT(2,1:3)];
% patch(coord(:,1),coord(:,2),coord(:,3),0.5)

coord=[sol2.AT(1,:); sol2.AT(2,:); [sol1.AT(1,:) 0]];
patch(coord(:,1),coord(:,2),coord(:,3),0.6)

coord=[sol2.AT(1,:); sol2.AT(3,:); sol3.AT(4,1:3)];
patch(coord(:,1),coord(:,2),coord(:,3),0.6)

% coord=[[sol1.AT(1,:) 0]; [sol1.AT(2,:) 0]; sol3.AT(3,end-2:end)];
% patch(coord(:,1),coord(:,2),coord(:,3),0.9)


alpha 0.2

axis equal
 grid on;
 
 
 %% Let us try now to do sth similar to Legrende polynomials (i.e. define
 %%them from a 2nd order differencial equation)
 close all;
 figure; hold on; 
  fplot(diff(sol1.A*T1,1), interv);
 
 figure; hold on;
%  fplot(diff(sol2.A*T2,1), interv);
 fplot(diff(sol2.A*T2,2), interv);
 
 
  figure; hold on;
%  fplot(diff(sol3.A*T3,1), interv);
%  fplot(diff(sol3.A*T3,2), interv);
  fplot(diff(sol3.A*T3,3), interv);
 
  figure; hold on;
    fplot(diff(sol5.A*T5,5), interv);
    
      figure; hold on;
    fplot(diff(sol7.A*T7,7), interv);
    
    %% Trying now with the inverses
    figure; hold on;
    sol5.Ai=inv(sol5.A);
    plot(sol5.Ai(:,1),'o');
    
        sol7.Ai=inv(sol7.A);
    plot(sol7.Ai(:,1),'o');
%     plot(sol5.Ai(:,2),'o');
%     plot(sol5.Ai(:,3),'o');
    
     sol3.Ai=inv(sol3.A);
    plot(sol3.Ai(:,1),'o');
%     plot(sol3.Ai(:,2),'o');
    
       p=polyfit([1:5]',sol5.Ai(1:end-1,1),1);
    plot(linspace(1,6),polyval(p,linspace(1,6)))
  %%  
    figure; hold on;
    sol3.Ai=inv(sol3.A);
    plot(sol3.Ai(:,1),'o');
    plot(sol3.Ai(:,2),'o');
    plot(sol5.Ai(:,3),'o');
    
    

 
    %%
    figure;
    plot(sol3.A(ceil(si,1)))
 %%
 figure
 Tinv1=[(1-t);1];
 Tinv2=[(1-t)^2 ;Tinv1];
 
 result=int((sol1.A(2,:)*T1)*sol2.A(1,:)*Tinv2,-1,t)
 fplot(result,interv)
 %%
 figure; hold on;
 tmp=eig(sol1.A);
 plot(real(tmp),imag(tmp),'o')
  tmp=eig(sol2.A);
 plot(real(tmp),imag(tmp),'o')
  tmp=eig(sol3.A);
 plot(real(tmp),imag(tmp),'o')
  tmp=eig(sol5.A);
 plot(real(tmp),imag(tmp),'o')
 

 %%
 a=sol2.AT(1,end-2:end-1);
 b=sol2.AT(2,end-2:end-1);
 a=[a 0];
 b=[b 0];
 angle = atan2(norm(cross(a,b)), dot(a,b))*180/pi
%%
figure
fplot(int(0.5*sol3.A(2,:)*T3-sol3.A(3,:)*T3,-1,t), interv);
legend('1','2','3')

%%
% disp("combining more")
% comb=combine2pol(polys1(2),polys1(3),0,sol.b,sol.c,sol.d);
% coeff_comb=vpa(coeffs(comb,t,'All'),4)
% fplot(coeff_comb*T2,[0,1]);


for i=1:2
    for j=1:2
        for k=1:3
            comb=combine2pol(polys1(i),polys1(j),a,b,c,d);
            coeff_comb=vpa(coeffs(comb,t,'All'),4);
            sol=solve(coeff_comb==sol2.A(k,:));
            disp(vpa([i j k 'ss' sol.b sol.c sol.d],4))
        end
    end
end

disp("=======================================")
% disp("combining first")
% comb=combine2pol(polys2(1),polys2(2),sol.a,sol.b,sol.c,sol.d);
% coeff_comb=vpa(coeffs(comb,t,'All'),4)
% 
% disp("combining more")
% comb=combine2pol(polys2(1),polys2(3),sol.a,sol.b,sol.c,sol.d);
% coeff_comb=vpa(coeffs(comb,t,'All'),4)

% T=[t*t*t t*t t 1]';
vpa(sol2.A,4)
vpa(coeff_comb,4);
vpa(sol3.A,4)

figure; hold on

fplot(sol2.A*T2, interv);
legend('1','2','3')
ylim([0,1]);
figure
fplot(sol3.A*T3, interv);
legend('1','2','3','4')
ylim([0,1]);
figure
fplot(sol5.A*T5, interv);
ylim([0,1]);
legend('1','2','3','4')

%%
figure;
fplot(sqrt(sol2.A*T2), interv);
axis equal

figure;
fplot(sqrt(sol3.A*T3), interv);
axis equal


figure;
fplot(sqrt(sol5.A*T5), interv);
axis equal

%%

syms B [4 4] real
syms C [4 4] real

A2=[zeros(4,2) sol2.A];

B=repmat(B(1,:),size(B,1),1); %B has the same row repeated
C=repmat(C(1,:),size(C,1),1); %B has the same row repeated


% (B*T2).*(sol2.A*T2) + (C*T3).*(sol3.A*T3) 
%%
syms a b c d f g h real
% comb=(a*t^2+ b*t +c)*sol2.A(1,:)*T2   +   (d*t +f)*sol3.A(1,:)*T3;
comb=(a*t^2+ b*t +c)*sol1.A(1,:)*T1   +  (d*t +f)*sol2.A(1,:)*T2  +  (g*t +h)*sol2.A(2,:)*T2;
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb==sol3.A(2,:));

% b=sol.b;
% c=sol.c;
d=sol.d;
f=sol.f;
g=sol.g;
h=sol.h;

comb=(a*t^2+ b*t +c)*sol1.A(2,:)*T1   +  (d*t +f)*sol2.A(2,:)*T2  +  (g*t +h)*sol2.A(3,:)*T2;
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb(1:3)==sol3.A(3,1:3))
%%
clc;
syms a1 b1 c1 d1 f1 a2 b2 c2 d2 f2
combA=(c1*t^2 + d1*t +f1)*sol1.A(1,:)*T1  +  (a1*t +b1)*sol2.A(1,:)*T2 +...
      (c2*t^2 + d2*t +f2)*sol1.A(2,:)*T1  +  (a2*t +b2)*sol2.A(2,:)*T2 ;
coeff_combA=vpa(coeffs(combA,t,'All'),3);

combB=(c1*t^2 + d1*t +f1)*sol1.A(2,:)*T1  +  (a1*t +b1)*sol2.A(2,:)*T2 +...
                  1.0                     +  (a2*t +b2)*sol2.A(3,:)*T2 ;
coeff_combB=vpa(coeffs(combB,t,'All'),3);

combC=            1.0                     +  (a1*t +b1)*sol2.A(3,:)*T2 +...
                  1.0                     +        1.0  ;
coeff_combC=vpa(coeffs(combC,t,'All'),3);


sol=solve([coeff_combA==sol3.A(2,:),coeff_combB==sol3.A(3,:),coeff_combC(1:2)==sol3.A(4,1:2)] )

%%
b=sol.b; c=sol.c; d=sol.d; f=sol.f;


comb=(c*t^2 + d*t +f)*sol1.A(2,:)*T1  +  (a*t +b)*sol2.A(2,:)*T2 ;
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb(1)==sol3.A(2,1))

coeff_comb=vpa(coeffs(subs(comb,a, sol),t,'All'),3)


%%
solut(1,2)
%% POR AHORA ESTA ES LA QUE FUNCIONA MEJOR
figure
clf; hold on;

%%%%%%%% a        b         c       d       f        g       h
sol=[ 0.1383, -4.194e-6, 0.1687, 0.8036, 0.4745, -0.8036, 0.4746];
a=sol(1);
b=sol(2);
c=sol(3);
d=sol(4);
f=sol(5);
g=sol(6);
h=sol(7);

comb=(a*t^2+ b*t +c)*solut(1,2)  +  (d*t +f)*solut(2,2)  +  (g*t +h)*solut(2,3);
fplot(comb,interv);
comb=(a*t^2+ b*t +c)*solut(1,1)  +  (d*t +f)*solut(2,1)  +  (g*t +h)*solut(2,2);
fplot(comb,interv);


comb=(a*t^2+ b*t +c)*solut(2,3)  +  (d*t +f)*solut(3,3)  +  0.0*(g*t +h)*solut(3,4);
fplot(comb,interv);


% comb=(a*t^2+ b*t +c)*solut(1,1)  +  (d*t +f)*(0.5-0.8*t)  + (g*t +h)*solut(2,1);
fplot(comb,2*interv,'-','LineWidth',4)


% comb=(a*t^2+ b*t +c)*solut(2,2)  +  (d*t +f)*solut(3,2) +  (g*t +h)*solut(3,3);
% fplot(comb,interv)

% comb=(a*t^2+ b*t +c)*solut(3,2)   +  (d*t +f)*solut(3,3)  +  (g*t +h)*solut(3,4);
% fplot(comb,interv)

% comb=0*(a*t^2+ b*t +c)*sol2.A(2,:)*T2   +  1*(d*t +f)*sol2.A(2,:)*T2  +  (g*t +h)*sol2.A(1,:)*T2;
% fplot(comb,interv)


% comb=0*(a*t^2+ b*t +c)*sol2.A(1,:)*T2   +  0*(d*t +f)*sol3.A(1,:)*T3  +  (g*t +h)*sol3.A(2,:)*T3;
% fplot(comb,interv)

% comb=(a*t^2+ b*t +c)*sol1.A(2,:)*T1   +  (d*t +f)*sol2.A(2,:)*T2  +  (g*t +h)*sol2.A(3,:)*T2;
% fplot(comb,interv)
% 
% comb=0*(a*t^2+ b*t +c)*sol1.A(1,:)*T1   +  0*(d*t +f)*sol2.A(1,:)*T2  +  (g*t +h)*sol1.A(1,:)*T1;
% fplot(comb,interv)
% 
% comb=0*(a*t^2+ b*t +c)*sol1.A(1,:)*T1   +  (d*t +f)*sol1.A(1,:)*T1  +  (g*t +h)*sol1.A(2,:)*T1;
% fplot(comb,interv)
% 
% comb=0*(a*t^2+ b*t +c)*sol1.A(1,:)*T1   +  (d*t +f)*sol1.A(1,:)*T1  +  (g*t +h)*sol1.A(2,:)*T1;
% fplot(comb,interv)

fplot(sol3.A*T3 ,'--', interv)
fplot(sol3.A*T3 ,'--', interv)
ylim([0,1])
%%
syms a b c d f g h k r s real
comb=(a*t^2+ b*t +c)*sol1.A(1,:)*T1 + (d*t^2+ f*t +g)*sol1.A(2,:)*T1  +  (h*t +k)*sol2.A(1,:)*T2  +  (r*t +s)*sol2.A(2,:)*T2;
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb==sol3.A(2,:));
h=sol.h; k=sol.k; r=sol.r; s=sol.s;


comb=(a*t^2+ b*t +c)*sol1.A(2,:)*T1 + 0*(d*t^2+ f*t +g)*sol1.A(2,:)*T1  +  (h*t +k)*sol2.A(2,:)*T2  +  (r*t +s)*sol2.A(3,:)*T2;
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb==sol3.A(3,:))
h=subs(h,[c d f g], [sol.c sol.d sol.f sol.g]);
k=subs(k,[c d f g], [sol.c sol.d sol.f sol.g]);
r=subs(r,[c d f g], [sol.c sol.d sol.f sol.g]);
s=subs(s,[c d f g], [sol.c sol.d sol.f sol.g]);
c=sol.c; d=sol.d; f=sol.f; g=sol.g;


comb=0*(a*t^2+ b*t +c)*sol1.A(2,:)*T1 + 0*(d*t^2+ f*t +g)*sol1.A(2,:)*T1  +  (h*t +k)*sol2.A(3,:)*T2  +  0*(r*t +s)*sol2.A(3,:)*T2;
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb(1:2)==sol3.A(4,1:2))

%% 1(i-3) + 2(i-3) == 2(i)  NO FUNCIONA
syms a b c d f g h real
comb=(a*t^2+ b*t +c)*sol1.A(1,:)*T1   +  (d*t^2 +f*t+g)*sol1.A(2,:)*T1; 
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb==sol3.A(2,:))

c=sol.c;
d=sol.d;
f=sol.f;
g=sol.g;

comb=(a*t^2+ b*t +c)*sol1.A(2,:)*T1   +  (d*t^2 +f*t+g)*sol1.A(1,:)*T1; 
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb==sol3.A(1,:))

%% 1(i-1) + 2(i-1) == 2(i)  NO FUNCIONA
clc;
%https://en.wikipedia.org/wiki/Gegenbauer_polynomials
syms a b c d f g h real
comb=(d*t +f)*sol2.A(1,:)*T2  +  (g*t +h)*sol2.A(2,:)*T2; 
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb==sol3.A(2,:))

d=sol.d;
f=sol.f;
g=sol.g;
h=sol.h;

comb=(1+d*t +f)*sol2.A(2,:)*T2  +  (1+g*t +h)*sol2.A(3,:)*T2; 
coeff_comb=vpa(coeffs(comb,t,'All'),3)
sol3.A(3,:)
% sol=solve(coeff_comb==sol3.A(3,:))
%%
clf
figure; hold on;
fplot((sol2.A*T2), interv,'r');
axis equal

fplot((sol3.A*T3), interv,'k');
axis equal


fplot((sol5.A*T5), interv,'b');
axis equal

fplot(-0.5*(t-1),interv,'m');
fplot(-0.5*(-t-1),interv,'m');

%%
clc;
figure; hold on
syms a b c real
comb=c*solut(2,1)+(b*t^2+a*t)*solut(1,1);%a*solut(2,1)+
coeff_comb=vpa(coeffs(comb,t,'All'),3);
sol=solve(coeff_comb(1:3)==sol3.A(1,1:3))

a=sol.a;
b=sol.b;
c=sol.c'
comb=(b*t^2+a*t+c)*solut(1,1);
coeff_comb=vpa(coeffs(comb,t,'All'),3);

fplot(solut(3,1),interv,'--')

fplot(comb,interv);
s=solve(comb==0);
vpa(s,4)
%%
% close all

figure; hold on
fplot(diff(sol2.A*T2,t), interv);
legend('1','2','3')

figure
fplot(diff(sol3.A*T3,t), interv);
legend('1','2','3')

figure
fplot(int(sol2.A*T2,-1,t), interv);
legend('1','2','3')

figure
fplot(int(sol3.A*T3,-1,t), interv);
legend('1','2','3')

figure;


figure; hold on
sphere
alpha 0.2
shading interp
fplot3(sqrt(sol2.A(1,:)*T2),sqrt(sol2.A(2,:)*T2),sqrt(sol2.A(3,:)*T2), interv)
xlabel('x');ylabel('y');zlabel('z');
axis equal

figure;
fplot3(sqrt(sol2.A(1,:)*T2),sqrt(sol2.A(2,:)*T2),sqrt(sol2.A(3,:)*T2), interv)
xlabel('x');ylabel('y'); zlabel('z');
axis equal;

figure; hold on;
fplot(sqrt(sol2.A(1,:)*T2),sqrt(sol2.A(2,:)*T2), interv)
xlabel('x');ylabel('y');
axis equal;

% figure;
fplot(sqrt(sol3.A(1,:)*T3),sqrt(sol3.A(2,:)*T3), interv)
xlabel('x');ylabel('y');
axis equal;

% figure;
fplot(sqrt(sol5.A*T5), interv)
axis equal;


figure;
fplot((sol3.A(1,:)*T3),(sol3.A(2,:)*T3), interv)
xlabel('x');ylabel('y');zlabel('z');
axis equal;


figure; hold on
fplot(sqrt(sol1.A(1,:)*T1),sqrt(sol1.A(2,:)*T1), interv); axis equal;
figure
fplot(sqrt(sol3.A*T3) + sqrt(sol3.A(3,:)*T3),sqrt(sol3.A(2,:))+sqrt(sol3.A(4,:)*T3), interv)

axis equal

%% Note that the objective function we're trying to minimize is the same one as the wronskian of the polynomials
figure; hold on;

comb=solut(3,1)+ diff(solut(3,1)) + diff(solut(3,1),2) + diff(solut(3,1),3);
vpa(comb)
fplot(comb,interv)

comb=solut(3,2)+ diff(solut(3,2)) + diff(solut(3,2),2) + diff(solut(3,2),3);
vpa(comb)
fplot(comb,interv)
%%

figure;
fplot(sqrt(sol2.A(1,:)*T2)/sqrt(sol2.A(3,:)*T2), interv)
% fplot(sqrt(sol5.A(1,:)*T5)/sqrt(sol5.A(4,:)*T5), interv)

%%
clf
figure;  hold on;
tmp=0:0.01:1;
theta=2*pi*tmp;

polarplot(theta,subs(sol3.A*T3,tmp),'r');

polarplot(theta,subs(sol2.A*T2,tmp),'k'); 

polarplot(theta,subs(sol5.A*T5,tmp),'b'); 
%%
w1=wronMatrix(1);
w2=wronMatrix(2);
w3=wronMatrix(3);
w5=wronMatrix(5);
% w3=double(subs(w3,t,0));

w2=subs(w2,t,0.0); 
w3=subs(w3,t,0.0); %shouldn't depend on t (the wronskian does NOT depend on t)

det(subs(w,t,2));
%%
figure; hold on;

fplot(w2(1,3)/w2(1,2))

solve(w2(1,3)==0)
solve(w2(1,2)==0)


fplot(w3(1,4)/w3(1,3))
solve(w3(1,4)==0)
solve(w3(1,3)==0)

%
figure
fplot(diff(solut(3,2)/(solut(3,2)+solut(3,1))),interv)

tmp=[zeros(1,3); normalize(w2(1,:))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','green','LineStyle','-','LineWidth',2 )
tmp=[zeros(1,3); normalize(sol3.AT(2,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','green','LineStyle','-')
tmp=[zeros(1,3); normalize(sol3.AT(3,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','green','LineStyle','-','LineWidth',2  )
tmp=[zeros(1,3); normalize(sol3.AT(4,1:3))];
line(tmp(:,1),tmp(:,2),tmp(:,3),'Color','green','LineStyle','-' )
%%
clc
% interv=[0,1];
figure; hold on;
% syms theta
% a=cos(theta)
T3cos=[sin(t)^3 sin(t)^2 sin(t)^1 1]';

T5cos=[sin(t)^5 sin(t)^4  T3cos']';

% fplot(sol3.A*T3cos,10*interv,'--');

T2cos=[cos(t)^2 cos(t)^1 1]';

fplot(sol5.A*T5cos,interv,'--');
% figure;
% fplot(sol3.Abz*T3cos,10*interv,'--');
%%
figure; hold on
% fplot(solut(3,1)+solut(3,3),interv)
fplot(solut(3,2)+solut(3,4),interv)
% fplot(0.5*(legendreP(3,t)+1),interv,'--')


a=0.50
offset=gegenbauerC(3,a,-1);
scaling=1/(gegenbauerC(3,a,1)-offset);
fplot(scaling*(gegenbauerC(3,a,t)-offset),interv,'--')

fplot(solut(2,1)+solut(2,3),interv)
% fplot(0.5*(legendreP(2,t)+1),interv,'--')

offset=gegenbauerC(5,a,-1);
scaling=1/(gegenbauerC(5,a,1)-offset);
fplot(scaling*(gegenbauerC(5,a,t)-offset),interv,'--')

fplot(solut(5,2)+solut(5,4)+solut(5,6),interv)
% fplot(0.5*(legendreP(5,t)+1),interv,'--')
%%
figure; hold on;


fplot(solut(3,1)+solut(3,4),interv)

fplot(solut(3,2)+solut(3,3),interv)

fplot(solut(5,1)+solut(5,6),interv,'--')
% fplot(0.5*(legendreP(5,t)+1),interv,'--')
% fplot(solut(3,2)+solut(3,4),interv)
%%
sol5.A_mod=[sol5.A(1,:);sol5.A(3,:);sol5.A(5,:)];

sol3.A_mod=[sol3.A(1,:);sol3.A(3,:)];

%%
inv5=inv(sol5.A)
x=1:6;
x=x-(max(x)-min(x)+2)/2
y=inv5(:,4)

p=polyfit(x,y,6)

x1 = linspace(min(x),max(x));
y1 = polyval(p,x1);
figure
plot(x,y,'o')
hold on
plot(x1,y1)
hold off


%%
figure; hold on;

derivadas=[]
for i=1:5
    derivadas=[derivadas (1/factorial(i))*diff(solut(5,3),i)];
end

fplot(sum(derivadas),interv)
% 
% p1=solut(5,1);
% p1p=diff(p1);
% p1pp=0.5*diff(p1,2);
% p1ppp=(1/6)*diff(p1,3);
% p1pppp=(1/factorial(4))*diff(p1,4);
% p1ppppp=(1/factorial(5))*diff(p1,5);
% 
% p10=double(subs(p1,t,0));
% p1p0=double(subs(p1p,t,0));
% p1pp0=double(subs(p1pp,t,0));
% p1ppp0=double(subs(p1ppp,t,0));
% p1pppp0=double(subs(p1pppp,t,0));
% p1ppppp0=double(subs(p1ppppp,t,0));
% 
% 
% suma1=p10+p1p0+p1pp0+p1ppp0+p1pppp0+p1ppppp0;



% suma2=p1p+p1ppp;
% fplot(p1,interv)
% fplot(p1p,interv)
% fplot(p1pp,interv)
% fplot(p1ppp,interv)
% fplot(p1pppp,interv)

% fplot(p1,interv);
% fplot(p1p,interv);
% fplot(p1pp,interv);
% fplot(p1ppp,interv);
% fplot(p1pppp,interv);
% fplot(p1ppppp,interv);

% fplot(p1p+p1pp+p1ppp+p1pppp+p1ppppp,interv,'--');

%%
figure; hold on
p1=sol5.A(1,:);

fplot(solut(3,1),interv)

fplot(p1*T5,interv)
permut1=[p1(1) p1(3) p1(5) p1(2) p1(4) p1(6)];
fplot(permut*T5,interv,'--');
fplot(permut*T5,interv,'--');
% fplot(suma2)
%%
figure;
fplot((-0.5*t+0.5)*sol2.Al(2,:)*T2+(0.5*t+0.5)*sol2.Al(1,:)*T2,interv);
hold on;
fplot(sol3.A(2,:)*T3,interv,'--')
%%
figure; syms tt; hold on
fplot(0.501*subs(sol3.Al(3,:)*T3+0.31,t,tt+0.0745),interv);
fplot(sol3.A(3,:)*T3,interv,'--')
ylim([0,1])

%%
figure; hold on; fplot(sol3.A(3,:)*T3,interv,'--')
local_extr_x=solve(diff(sol3.Al(3,:)*T3)==0)
local_min_x=double(vpa(min(local_extr_x)))
local_min_y=polyval(sol3.Al(3,:),local_min_x);
tmp=solve(sol3.Al(3,:)*T3==local_min_y, 'maxdegree', 5)
shift=max(tmp)-1;
fplot(subs(sol3.Al(3,:)*T3-local_min_y,t,tt+shift),interv);

%%
delta_x=-0.773548602-local_min_x;

factor_x=(-0.773548602)/local_min_x

%%


%%
%%vamos ahora a por el 4
local_extr_x=solve(diff(sol3.Al(4,:)*T3)==0);
local_min_x=double(vpa(max(local_extr_x)));

delta_x+local_min_x
factor_x*local_min_x
%%



pol_x=[0 0 1 0]';%[a b c d]
pol_y=[0 1 0 0]';%[a b c d]
pol_z=[1 0 0 0]';%[a b c d]

plot_convex_hull(pol_x,pol_y,pol_z,sol3.A','red')
fplot3(pol_x'*T3, pol_y'*T3, pol_z'*T3,[0 1],'r','LineWidth',3)
%  axis equal
%%
sol5.A_norm=sol5.Ai./repmat(sol5.Ai(:,1),1,6)
plot(sol5.A_norm,'o')
%%

%%
close all
vtet_x = [1 0 1; 0 0 0; 0 0 0]*1.5;
vtet_y = [0 0 0; 1 0 1; 0 0 0]*1.5;
vtet_z = [0 1 0; 0 0 0; 1 0 1]*1.5;

v1=[vtet_x(1) vtet_y(1) vtet_z(1)]';
v2=[vtet_x(2) vtet_y(2) vtet_z(2)]';  
v3=[vtet_x(3) vtet_y(3) vtet_z(3)]';
v4=[vtet_x(4) vtet_y(4) vtet_z(4)]'; 

vx=[1 0 0 0]';
vy=[0 1 0 0]';
vz=[0 0 1 0]';
% 
vx=[1           -1          0         0 ]';
vy=[-1/sqrt(3)  -1/sqrt(3)  2/sqrt(3)         0]';
% vz=[-1/sqrt(6)  -1/sqrt(6)   -1/sqrt(6)       3/sqrt(6)]';
vz=[0  0   0       4/sqrt(6)]';

%With these ones its aliged in the good axis
tmp=2 %sqrt(3)*sqrt(2)
vx=tmp*[1 -1  -1   1 ]';
vy=tmp*[1 -1  1   -1 ]';
vz=tmp*[1  1  -1  -1 ]';

%With these ones its aliged in the good axis
% vx=[0           1         -1         0 ]';
% vy=[sqrt(2)     0         0         sqrt(2)]';%0.5468*sqrt(2)*
% vz=[-1 0 0 1]';

%(to have volume=1) With these ones its aliged in the good axis
% a=2.03964 %raiz cubica de 6raiz2
% vx=a/2*[0           1         -1         0 ]';
% vy=a/2*[sqrt(2)     0         0         sqrt(2)]';%0.5468*sqrt(2)*
% vz=a/2*[-1 0 0 1]';

    v1=[vx(1) vy(1) vz(1)]'
    v2=[vx(2) vy(2) vz(2)]'  
    v3=[vx(3) vy(3) vz(3)]'
    v4=[vx(4) vy(4) vz(4)]'  

pol_x=sol3.A'*vx;
pol_y=sol3.A'*vy;
pol_z=sol3.A'*vz;

plot_convex_hull(pol_x,pol_y,pol_z,sol3.A','red')
fplot3(pol_x'*T3, pol_y'*T3, pol_z'*T3,[-1 1],'r','LineWidth',3)
axis equal
grid
% fplot3(-pol_x'*T3, pol_y'*T3, pol_z'*T3,[-1 1],'-r','LineWidth',3)
% fplot3(-pol_x'*T3, -pol_y'*T3, pol_z'*T3,[-1 1],'-r','LineWidth',3)
% fplot3(-pol_x'*T3, -pol_y'*T3, -pol_z'*T3,[-1 1],'-r','LineWidth',3)
% fplot3(-pol_x'*T3, pol_y'*T3, -pol_z'*T3,[-1 1],'-r','LineWidth',3)

poly=[pol_x'*T3; pol_y'*T3; pol_z'*T3];

center=[mean(vx),mean(vy),mean(vz)]';




center_side=(v2+v3)/2

scatter3(center(1),center(2),center(3),450,'Filled','red')
scatter3(center_side(1),center_side(2),center_side(3),45,'Filled')

poly_0=double(subs(poly,t,0.0));

poly_1=double(subs(poly,t,1));
poly_m1=double(subs(poly,t,-1));

poly_05=double(subs(poly,t,0.5));
poly_m05=double(subs(poly,t,-0.5));


tmp=[poly_m1 poly_1 center_side];

patch(tmp(1,:), tmp(2,:), tmp(3,:),[0.5, 0.5, 0.5],'FaceAlpha',.2)

for i=1:numel(sol3.rootsA)
    tmp=sol3.rootsA(:);
    poly_tg=double(subs(poly,t,tmp(i)))
    scatter3(poly_tg(1),poly_tg(2),poly_tg(3),100,'Filled','blue')
end

poly_tg1=double(subs(sol5.rootsA,t,0.5));

scatter3(poly_0(1),poly_0(2),poly_0(3),460)
scatter3(poly_05(1),poly_05(2),poly_05(3),100,'Filled','green')
scatter3(poly_m05(1),poly_m05(2),poly_m05(3),100,'Filled','green')



samples_t=-1:0.01:1;
samples_poly=subs(poly,t,samples_t);
centroid_curve=sum(samples_poly,2)/length(samples_t);
scatter3(centroid_curve(1),centroid_curve(2),centroid_curve(3),405,'Filled','blue'); 


vecnorm(double(samples_poly-centroid_curve));

% [azimuth,elevation,r] = cart2sph(samples_poly(1,:),samples_poly(2,:),samples_poly(3,:));

% [theta,rho,z] = cart2pol(samples_poly(1,:),samples_poly(2,:),samples_poly(3,:));

coeff_x=coeffs(poly(1,:),'All'); coeff_x=[zeros(1,4-length(coeff_x)),coeff_x];
coeff_y=coeffs(poly(2,:),'All'); coeff_y=[zeros(1,4-length(coeff_y)),coeff_y];
coeff_z=coeffs(poly(3,:),'All'); coeff_z=[zeros(1,4-length(coeff_z)),coeff_z];

M=double([coeff_x;  coeff_y; coeff_z]);


%%
a1=M(1,1); c1=M(1,3);
a2=M(2,2); d2=M(2,4);
a3=M(3,1); c3=M(3,3);


ax=a3/a2;
bx=-(a3/a2)*d2+c3;
ay=(a1/a2);
by=-(a1/a2)*d2+c1;

% x(ax*ybx)=z(ay*y+by);
% syms x y
% f = @(x,y,z) x.^2 + y.^2 - z.^2;
fimplicit3( @(x,y,z) x.*(ax*y+bx)-z.*(ay*y+by));
xlim([-2 2])
%%


% M=M./repmat(M(:,1),1,4)

xp=(v2-center_side); xp=xp/norm(xp);
yp=(center-center_side); yp=yp/norm(yp);
zp=cross(xp,yp); zp=zp/norm(zp);

% quiver(xp(1),p1(2),dp(1),dp(2),0)
% arrow3d(center_side',(center_side+xp)')
% arrow3d(center_side',(center_side+yp)')
% arrow3d(center_side',(center_side+zp)')

% arrow3d(center_side',(center_side+double(subs(diff(poly,t),t,0)))')

R=[xp yp zp ]; %transformation matrix
o_T_n=[R center_side]; o_T_n=[o_T_n; [0 0 0 1]];  %n_P=n_T_o*o_P  new and old basis

n_polyhomog=inv(o_T_n)*[poly; 1];
n_poly=n_polyhomog(1:3);

% M=double([coeffs(n_poly(1,:),'All');coeffs(n_poly(2,:),'All'); coeffs(n_poly(3,:),'All') ]);

center12=(v1+v2)/2.0;
x_line=[center12(1) v4(1)];
y_line=[center12(2) v4(2)];
z_line=[center12(3) v4(3)];
plot3(x_line,y_line,z_line)

tmp_proj=center;
for i=1:(length(samples_poly)-1)
    tmp1=samples_poly(:,i);
    tmp2=samples_poly(:,length(samples_poly)-i);
   direc=(tmp1-tmp2);
   vertex_line=tmp2+direc;
   x_line=[tmp_proj(1) vertex_line(1)];
   y_line=[tmp_proj(2) vertex_line(2)];
   z_line=[tmp_proj(3) vertex_line(3)];
   plot3(x_line,y_line,z_line,'b')
end

% figure; hold on;
% tmp_proj=center;
% scale=r
% for i=1:length(samples_poly)
%    direc=(samples_poly(:,i)-tmp_proj);
%    vertex_line=tmp_proj+scale*direc/norm(direc);
%    x_line=[tmp_proj(1) vertex_line(1)];
%    y_line=[tmp_proj(2) vertex_line(2)];
%    z_line=[tmp_proj(3) vertex_line(3)];
%    plot3(x_line,y_line,z_line,'b')
%    
% %       vertex_line=tmp_proj-scale*direc/norm(direc);
% %    x_line=[tmp_proj(1) vertex_line(1)];
% %    y_line=[tmp_proj(2) vertex_line(2)];
% %    z_line=[tmp_proj(3) vertex_line(3)];
% %    plot3(x_line,y_line,z_line,'ob')
% end

% tmp_proj=v4;
% for i=1:length(samples_poly)
%    direc=(samples_poly(:,i)-tmp_proj);
%    vertex_line=tmp_proj+scale*direc/norm(direc);
%    x_line=[tmp_proj(1) vertex_line(1)];
%    y_line=[tmp_proj(2) vertex_line(2)];
%    z_line=[tmp_proj(3) vertex_line(3)];
%    plot3(x_line,y_line,z_line,'g')
% end

% [X,Y,Z] = sphere;
% hold on
% r = 0.6473;
% X2 = X * r;
% Y2 = Y * r;
% Z2 = Z * r;
% surf(X2+center(1),Y2+center(2),Z2+center(3))
% alpha 0.2
axis equal

n_v1=inv(o_T_n)*[v1;1]; n_v1=n_v1(1:3);
n_v2=inv(o_T_n)*[v2;1]; n_v2=n_v2(1:3);
n_v3=inv(o_T_n)*[v3;1]; n_v3=n_v3(1:3);
n_v4=inv(o_T_n)*[v4;1]; n_v4=n_v4(1:3);

[n_v1 n_v2 n_v3 n_v4];


% figure; hold on;
% fplot(n_poly,interv)
% legend('x','y','z')
% 
% x=(n_poly(2,:)-1.227)./1.226;
% y=n_poly(1,:)*0.648;
% fplot(x,y,interv);
% z=n_poly(3,:);
% 
% % fplot(x*x*x*x+1*(y*y-x*x),interv)
% figure;hold on
% % fplot(y*y-x*x*(1-x*x),interv)
% fplot(x,y,interv);
% fimplicit(@(x,y) (y*y)-x*x*(x+1));
% xlim([-1,0])
% ylim([-0.5,0.5])

n_samples_poly=[];
for i=1:size(samples_poly,2)
    o_tmp=[samples_poly(:,i); 1];
    n_samples_poly=[n_samples_poly double(inv(o_T_n)*o_tmp)];
end


figure;hold on;
tmp_x=n_samples_poly(2,:);
tmp_y=n_samples_poly(1,:);
% tmp_z=abs(sqrt(1-tmp_x.*tmp_x-tmp_y.*tmp_y));
% plot3(tmp_x,tmp_y,tmp_z);
plot(tmp_x,tmp_y);
% fimplicit(@(x,y) y*y-x*x*(x+1));
axis equal

% [theta,rho,z] = cart2pol(n_samples_poly(1,:),n_samples_poly(2,:),n_samples_poly(3,:));
% figure;
% plot(theta)

%%
% Si uso este sistema de coordenadas, aparece la basis!!!
xpp=v1-center; xpp=xpp/norm(xpp);
ypp=v2-center;  ypp=ypp/norm(ypp);
zpp=v3-center;  zpp=zpp/norm(zpp);
kpp=v4-center;  kpp=kpp/norm(kpp);

%en uno de los vertices
% xpp=v2-v1; xpp=xpp/norm(xpp);
% ypp=v3-v1;  ypp=ypp/norm(ypp);
% zpp=v4-v1;  zpp=zpp/norm(zpp);

% apuntando a cada plano
% xpp=sum([v1,v2,v3],2)/3-center; xpp=xpp/norm(xpp);
% ypp=sum([v1,v2,v4],2)/3-center;  ypp=ypp/norm(ypp);
% zpp=sum([v1,v3,v4],2)/3-center;  zpp=zpp/norm(zpp);
% 
% xpp=(v2-center_side); xpp=xpp/norm(xpp);
% ypp=(v1-center_side); ypp=ypp/norm(ypp);
% zpp=(v4-center_side); zpp=zpp/norm(zpp);

% center_poly1_and_m1=(poly_1+poly_m1)/2.0;
% xpp=(poly_1-poly_m1); xpp=xpp/norm(xpp);
% ypp=(center-center_side);  ypp=ypp/norm(ypp);
% zpp=cross(xpp,ypp);  zpp=zpp/norm(zpp);


figure; hold on;
nn_x= dot(repmat(xpp,1,size(samples_poly,2)),samples_poly-center);
nn_y= dot(repmat(ypp,1,size(samples_poly,2)),samples_poly-center);
nn_z= dot(repmat(zpp,1,size(samples_poly,2)),samples_poly-center);
nn_k= dot(repmat(kpp,1,size(samples_poly,2)),samples_poly-center);
plot(samples_t, nn_x)
plot(samples_t, nn_y)
plot(samples_t, nn_z)
plot(samples_t, nn_k)
plot(samples_t, nn_x+nn_y+nn_z+nn_k)

figure;

plot3(nn_x,nn_y,nn_z)

syms a1 c1 a2 d2 a3 c3;

n_poly_unk=[a1*t^3+c1*t; a2*t+d2; a3*t^3+c3*t];
n_v3= inv(o_T_n) *[v3; 1];n_v3=n_v3(1:3);
n_v2= inv(o_T_n) *[v2; 1];n_v2=n_v2(1:3);

n_poly_0=vpa(subs(n_poly,t,0));

solve([n_poly_0==subs(n_poly_unk,t,0), subs(diff(n_poly_unk,t),t,0.0)==(n_v3-n_v2)])

global x y z
syms x y z

curvem1=subs(n_poly_unk,t,-1);
curve1=subs(n_poly_unk,t,1);
eq1=plane_eq_from_three_points(v1,v3,v4,curvem1(1),curvem1(2),curvem1(3));
eq2=plane_eq_from_three_points(v1,v2,v3,curvem1(1),curvem1(2),curvem1(3));

eq3=plane_eq_from_three_points(v1,v2,v4,curve1(1),curve1(2),curve1(3));
eq4=plane_eq_from_three_points(v2,v3,v4,curve1(1),curve1(2),curve1(3));

s=solve([eq1==0, eq2==0])

%%
figure; hold on;
% vx=[sol3.A(1,1:3) 0]; vy=[sol3.A(2,1:3) 0]; vz=[sol3.A(3,1:3) 0];
vx=[sol3.A(1:3,1); 0]/sol3.A(1,4); vy=[sol3.A(1:3,2); 0]/sol3.A(2,4); vz=[sol3.A(1:3,3); 0]/sol3.A(3,4);
[k1,volume] = convhull(vx,vy,vz);
trisurf(k1,vx,vy,vz,'FaceColor','red','FaceAlpha',0.2)


% vx=sol2.A(1,:); vy=sol2.A(2,:);
% [k1,volume] = convhull(vx,vy);
% trisurf(k1,vx,vy,vz,'FaceColor','red','FaceAlpha',0.2)
% axis equal;

% https://math.stackexchange.com/questions/2686606/equation-of-a-plane-passing-through-3-points
function result=plane_eq_from_three_points(a,b,c,x,y,z)
%     global x y z
%     syms x y z
%     normal=cross((b-a),(c-a));
    A=[x y z 1; [a' 1]; [b' 1]; [c' 1] ];
    result=det(A)
end

function volume=plot_convex_hull(pol_x,pol_y,pol_z,A,color)
    cx=pol_x;
    cy=pol_y;
    cz=pol_z;

    vx=inv(A)*cx;
    vy=inv(A)*cy;
    vz=inv(A)*cz;

    v1=[vx(1) vy(1) vz(1)]'
    v2=[vx(2) vy(2) vz(2)]'  
    v3=[vx(3) vy(3) vz(3)]'
    v4=[vx(4) vy(4) vz(4)]'  

    %Hack to see what happens if I choose the first and last control points
%     if color=='b'
%         v1=[1 6 -4]';
%         v4=[3.5 3.7 -4.1]';
%         vx(1)=v1(1);
%         vy(1)=v1(2);
%         vz(1)=v1(3);
%         vx(4)=v4(1);
%         vy(4)=v4(2);
%         vz(4)=v4(3);
figure; hold on;
  
    plot3(v1(1),v1(2),v1(3),'-o','Color',color,'MarkerSize',10)
    plot3(v2(1),v2(2),v2(3),'-o','Color',color,'MarkerSize',10)
    plot3(v3(1),v3(2),v3(3),'-o','Color',color,'MarkerSize',10)
    plot3(v4(1),v4(2),v4(3),'-o','Color',color,'MarkerSize',10)
  %  end
    
         [k1,volume] = convhull(vx,vy,vz);
 
%     if color=='b'
    trisurf(k1,vx,vy,vz,'FaceColor',color)
   
    xlabel('x')
    ylabel('y')
    zlabel('z')
    alpha 0.1
%      end
    plot3(v1(1),v1(2),v1(3),'-o','Color',color,'MarkerSize',10)
    plot3(v2(1),v2(2),v2(3),'-o','Color',color,'MarkerSize',10)
    plot3(v3(1),v3(2),v3(3),'-o','Color',color,'MarkerSize',10)
    plot3(v4(1),v4(2),v4(3),'-o','Color',color,'MarkerSize',10)
%    
end

function result=wronMatrix(n)

global sol1 sol2 sol3 sol5 sol7 T1 T2 T3 T5


result=[];
switch n
    case 1
        tmp=sol1.A*T1;
    case 2
        tmp=sol2.A*T2;
    case 3
        tmp=sol3.A*T3;
    case 5
        tmp=sol5.A*T5;
    case 7
        tmp=sol7.A*T7;
    otherwise
        disp('NOT IMPLEMENTED')
        tmp="NOT IMPLEMENTED" 
end

for i=0:n
    disp('here');
    result=[diff(tmp,i),result];
end

result=vpa(result,3);

end

function result=solut(n,i)
global sol1 sol2 sol3 sol5 sol7 T1 T2 T3 T5

switch n
    case 1
        result=sol1.A(i,:)*T1;
    case 2
        result=sol2.A(i,:)*T2;
    case 3
        result=sol3.A(i,:)*T3;
    case 5
        result=sol5.A(i,:)*T5;
    case 7
        result=sol7.A(i,:)*T7;
    otherwise
        disp('NOT IMPLEMENTED')
        result="NOT IMPLEMENTED" 
end

end

function roots_transformed=transformTo01(roots)
    roots_transformed=zeros(size(roots));
    
    for i=1:size(roots,1)
        for j=1:size(roots,2)
            roots_transformed(i,j)=(roots(i,j)-(-1))/2;
        end
    end
end

function comb=combine2pol(pol1, pol2,a,b,c,d)
    global t
    comb=(a*t+b)*pol1+(c*t +d)*pol2;
end

function result=transformStructureTo01(sol)

syms tt real
syms t real
tt=2*(t-0.5);

deg=size(sol.A,2)-1;

T=[];
for i=0:(deg)
    T=[tt^i T];
end

transformed2=sol.A*T';

A=[];

for i=1:size(sol.A,1)
    A=[A ;double(vpa(coeffs(transformed2(i),'All')))];
end

rootsA=[];
for i=1:size(A,1)
    rootsA=[rootsA ; roots(A(i,:))'];
end
rootsA=double(real(rootsA));


result.A=A;
result.rootsA=rootsA;


end

function an=normalize(a)
an= a/norm(a);
end
function Abz=computeMatrixForBezier(degree)
syms t
 Abz=[];
tmp=bernsteinMatrix(degree, t);
    for i=1:length(tmp)
        Abz=[Abz; coeffs(tmp(i),t,'All')];

    end
end

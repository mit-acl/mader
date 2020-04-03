close all; clc; clear;
syms t

deg=3;

W=[];

B=sym('B',[((deg+1)/2),1],'real');
R=sym('R',[(deg+1)/2,(deg-1)/2],'real');

%Insert half of the polynomials
for i=1:((deg+1)/2)
    pol=-B(i)*(t-1);
    for j=1:(deg-1)/2
        pol=pol*((t-R(i,j))^2);
    end
    pol
    W=[W;pol];
end

%Insert the other half
W=[W;subs(W,t,-t)]

%Solve for the coefficients:
coeffic=coeffs(sum(W),t,'All'); %When using All, no flip!! (gives directly [a  b c d]

coeffic=[zeros(1,deg+1-length(coeffic)) coeffic];

solution=solve(coeffic==[zeros(1,deg) 1],B); %This is a linear system

B_solved=simplify(struct2array(solution)');
W=subs(W,B,B_solved);

%Create the A matrix:
A=[];
for i=1:length(W)
    tmp=coeffs(W(i),t,'All');
    A=[A; tmp];
end
%Compute the determinant
disp("Computing determinant")
determ=det(A);


disp("Computing gradient")
 grad=gradient(determ,R(:));

%  disp("Computing ND")
% [N, D]=numden(grad);
%  disp("Simplifying N")
% expand(simplify(N))

 disp("vpasolve")
R_solved=solve(grad==zeros(length(grad),1),R(:),'Real',true, 'MaxDegree', 5);
 
% 
% B_sdp=sdpvar((deg-1),1);
% 
% h=matlabFunction(determ);
% 
% 
% ceq=matlabFunction(B);
% 
% x0 = [-1,2,1.5];
% A = eye(;
% b = 1;
% fmincon(h, , )


% function W=createPol()
%     W=
% end

% %We can solve it manually as well:
% M=equationsToMatrix(coeffic==[zeros(1,deg) 1],B);
% %Remove the columns and the rows with all zeroes
% M( all( isAlways(M==0) ,2) ,:) = [];
% M( : ,all( isAlways(M==0) ,1)) = [];
% %Solve the system:
% solution_m=simplify(inv(M)*[zeros(1,size(M,1)-1) 1]');


% sym('R',[(deg-1),1]);
% A=sym('A',[length(B),length(B)]);
% A=equationsToMatrix(coeffic==[zeros(1,deg) 1],B);
% A=A(end-1:end, :)
% for i=1:length(B)
%     for j=1:length(B)
%     %tmp= coeffs(coeffic(i),B(j))
%     A(i,j)=diff(coeffic(i),B(j)); %Assumming it's linear
%     end
% end

close all; clc; clear;


deg=6; %must be par

deg_s=deg;
deg_t=deg-2;
size_w=(deg_s/2)+1;
size_v=(deg_t/2)+1;

% size_w=2;
% size_V=size_w-1;

% A1=sym('A1_%d%d',[size_w,size_w],'real');
% A2=sym('A2_%d%d',[size_w,size_w],'real');
% A3=sym('A3_%d%d',[size_w,size_w],'real');
% A4=sym('A4_%d%d',[size_w,size_w],'real');
% 
% B1=sym('B1_%d%d',[size_w,size_w],'real');
% B2=sym('B2_%d%d',[size_w,size_w],'real');
% B3=sym('B3_%d%d',[size_w,size_w],'real');
% B4=sym('B4_%d%d',[size_w,size_w],'real');

W1=sym('W1_%d%d',[size_w,size_w],'real');
% W2=sym('W2_%d%d',[size_w,size_w],'real');
% W3=sym('W3_%d%d',[size_w,size_w],'real');
% W4=sym('W4_%d%d',[size_w,size_w],'real');

V1=sym('V1_%d%d',[size_v,size_v],'real');
% V2=sym('V2_%d%d',[size_v,size_v],'real');
% V3=sym('V3_%d%d',[size_v,size_v],'real');
% V4=sym('V4_%d%d',[size_v,size_v],'real');

W1 = tril(W1,0) + tril(W1,-1).';
% W2 = tril(W2,0) + tril(W2,-1).';
% W3 = tril(W3,0) + tril(W3,-1).';
% W4 = tril(W4,0) + tril(W4,-1).';

V1 = tril(V1,0) + tril(V1,-1).';
% V2 = tril(V2,0) + tril(V2,-1).';
% V3 = tril(V3,0) + tril(V3,-1).';
% V4 = tril(V4,0) + tril(V4,-1).';


syms t real;
T2=[];
for i=1:size_w
T2=[t^(size_w-i); T2];
end

T1=[];
for i=1:size_v
T1=[t^(size_v-i); T1];
end

tmp=T2'*W1*T2 + (t+1)*(1-t)*T1'*V1*T1;

coeffic=coeffs(tmp,t,'All');
coeffic'

% %%
% % W1=A1'*A1;
% % W2=A2'*A2;
% % W3=A3'*A3;
% % W4=A4'*A4;
% % 
% % V1=B1'*B1;
% % V2=B2'*B2;
% % V3=B3'*B3;
% % V4=B4'*B4;
% 
% S=diag(ones(2*size_w-1,1),-1); %It has ones below the diagonal
% 
% P=eye(2*size_w,2*size_w)
% P=[P(1:2:size(P,1),:) ; P(2:2:size(P,1),:) ]% F is a permutation matrix. First odd rows, then even rows. 
% % P=[1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
% I=eye(2*size_w);
% 
% size(tilda(W1))
% 
% Ap=P*(S+I)*[tilda(W1) tilda(W2) tilda(W3) tilda(W4)] + P*(S-I)*[tilda(V1) tilda(V2) tilda(V3) tilda(V4)];
% 
% R=[P*(S+I) P*(S-I)];
% Rc=R(1:2,:); Rb=R(3:4,:);
% 
% K=[tilda(W1)  tilda(W2) ; tilda(V1)  tilda(V2)];
% 
% U=K*K';
% 
% 
% % C=simplify(Ap(1:2,1:2))
% % 
% % B=simplify(Ap(3:4,1:2))
% % 
% % C*B
% % determ=det(C)*det(B);
% % gradient(determ, A1(:))
% %%
% tilda(W1)
% 
% V=[0 0; 1 0];
% U=rot90(eye(2));
% 
% trace(V*U*W1)
% trace(U*W1)
% trace(V'*U*W1)
% 
% 
% function out=tilda(A)
% 
%     Ar=rot90(A);
%     out=[];
%     for i=(size(A,1)-1):-1:-size(A,1)
%         out=[out; sum(diag(Ar,i))];
%     end
% %https://www.mathworks.com/matlabcentral/answers/294766-most-efficient-way-to-sum-anti-diagonal-elements
% % [m,n] = size(A);
% % idx = hankel(1:m,m:(n-1)+m);
% % out = accumarray(idx(:),A(:));
% % out=flip(out);
% end
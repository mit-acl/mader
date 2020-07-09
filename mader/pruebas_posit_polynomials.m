%Jesus Tordesillas Torres, jtorde@mit.edu, February 2020
clc; close all; clear;
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

t=sym('t','real');
int_minA1=0;
int_maxA1=1;
int_minA2=-0.8;
int_maxA2=1;

% times=[1 t t*t t*t*t]';
% A1=getRandom(int_minA1,int_maxA1,4); %almost for sure will be full-rank
% A2=getRandom(int_minA2,int_maxA2,4); %almost for sure will be full-rank
% poly=times'*(A2'*A2-A1'*A1*(t*t-1))*times
% fplot(poly,[0 1])


times=[1 t]';
%S1=getRandom(int_minA1,int_maxA1,2); %almost for sure will be full-rank
%R1=getRandom(int_minA1,int_maxA1,2); %almost for sure will be full-rank

S1=[0.5280    0.0817;
    0.1851    0.4641];

R1=[0.0306    0.5579
    0.4350    0.6388];

M=[1 0; 0 0];
S2TS2=M-S1'*S1;
R2TR2=M-R1'*R1;

% R2=sqrtm(R2TR2); %Cuidado que esto NO te calcula A=sqrt(B) such that A'*A=B, sino A*A=B
% S2=sqrtm(S2TS2);
% if(isreal(R2)==false | isreal(S2)==false)
%     disp('Result not valid')
% end

%polyS1=times'*S1'*S1*times;
%polyR1=times'*R1'*R1*times;

polyS2=times'*S2TS2*times;
polyR2=times'*R2TR2*times;

%poly1=times'*(    t*(S1'*S1-R1'*R1) + R1'*R1       )*times;
poly2= t*polyS2 + (1-t)*polyR2;%   times'*(    t*(S2TS2-R2TR2) + R2TR2       )*times;


%A2=getRandom(int_minA2,int_maxA2,2); %almost for sure will be full-rank
%poly=times'*(A2'*A2-A1'*A1*(t*t-1))*times
figure; hold on;
%fplot(poly1,[0 1])
fplot(poly2,[0 1])
%fplot(poly1+poly2,[0 1])

figure; hold on;
% fplot(polyS1);hold on;
fplot(polyS2);
% fplot(polyR1);hold on;
fplot(polyR2);


function result=getRandom(a,b,n)
result= -a + (b-a)*rand(n,n);
end
close all; clc; clear;
t=[0:20];
y=rand(1,length(t));
x=rand(1,length(t));
x_spline=spline(t,x);
y_spline=spline(t,y);
% y_spline=tmp.coefs;

t=0:0.01:10
plot(ppval(x_spline,t), ppval(y_spline,t))
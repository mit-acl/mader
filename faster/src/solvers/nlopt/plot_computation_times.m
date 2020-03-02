clc; clear; close all;
A=dlmread('example.txt');
n_pol=A(:,1);
comp_time=1000*A(:,3);
delta=A(:,2)



figure
hAX=axes;                 % first axes, save handle
pos=get(hAX,'position')   % get the position vector
pos =[  0.1300    0.1100    0.7750    0.8150];
pos1=pos(2);              % save the original bottom position
pos(2)=pos(2)+pos1; pos(4)=pos(4)-pos1;  % raise bottom/reduce height->same overall upper position
set(hAX,'position',pos)   % and resize first axes
pos(2)=pos1; pos(4)=0.01; % reset bottom to original and small height
hAX(2)=axes('position',pos,'color','none');  % and create the second

xlabel(hAX(2),'Delta (s)')
set(hAX(2),'xcolor','r','ycolor','r')

plot(n_pol, comp_time, 'o', 'Parent', hAX(1))
plot(n_pol, comp_time, 'o', 'Parent', hAX(1))
xlabel(hAX(1),'Number of segments')
ylabel(hAX(1),'Comp time (ms)')
hAX(2).XLim=[min(delta), max(delta)]

set ( gca, 'xdir', 'reverse' )
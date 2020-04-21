clc; close all;clear;
set(0,'DefaultFigureWindowStyle','docked');
% xp=[1 2 4 5]; 
xp=linspace(-1,1,4)
% xp=[-1 -0.77 -0.77    1]
coeffs=lagrangePoly(xp)

xx=linspace(xp(1),xp(end),1000);
for i=1:numel(xp)
    plot(xx,polyval(coeffs(i,:),xx))
    if i==1
        hold on
    end
end
plot(xp,ones(size(xp)),'o'); plot(xp,zeros(size(xp)),'o')
legend
hold off
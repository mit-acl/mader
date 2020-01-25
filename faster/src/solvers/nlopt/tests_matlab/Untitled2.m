p=[9;-5];
R=1;
center=[0;0]
a=center - p;

d=R*norm(a)-a'*center
figure; hold on; 
fimplicit(@(x,y) a(1)*x +a(2)*y + d)
plot_circle(center(1), center(2), R)
plot(p(1), p(2),'ro')

axis equal

function h = plot_circle(x,y,r)
%hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit);
h=[xunit; yunit];
% hold off
end
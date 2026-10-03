x=linspace(0,2*pi,50);
y=exp(x).*sin(x);
p=polyfit(x,y,7);
yy=polyval(p,x);
figure


plot(x,y,'bo');
hold on
plot(x,yy,'r');

x1=linspace(0,4*pi,50);
y1=exp(x1).*sin(x1);
p1=polyfit(x1,y1,8);
yy1=polyval(p1,x);
figure


plot(x1,y1,'b+');
hold on
plot(x1,yy1,'r');
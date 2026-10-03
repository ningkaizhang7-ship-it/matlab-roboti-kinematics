x=linspace(-10,10,100);
y=linspace(-10,10,100);
[X,Y]=meshgrid(x,y);
Z=sin((X^2+Y^2)^0.5)/(X^2+Y^2)^0.5;
surf(X,Y,Z);

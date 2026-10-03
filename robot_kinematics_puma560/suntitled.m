% A=[0 1 0; 0 0 1; -5 25 -5];
% B=[0;25;-120];
% C=[1 0 0];
% D=0;
% G=ss(A,B,C,D)
% [num,den]=ss2tf(A,B,C,D)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
% G1=tf(num,den)
% [z,p,k]=ss2zp(A,B,C,D)
% G2=zpk(z,p,k)
% 
% num=1
% den=[6 6 6]
% t=0:0.1:10
% % impulse(num,den,t)
% % grid on
% 
% G=tf(num,den)
% step(G,t)
% grid on
% G=tf(100,[1 3 100]);
% step(G)
% Mp=(1.62-1)/1
% ts=2.59
% tr=0.174
% tp=0.307
% ess=0.02
% G=tf(100,[1,3,100]);
% step(G);[y,t]=step(G); 
% C=dcgain(G);
% [y,k]=100*(y-C)/C
% num=11;den=[1 5 7 9 11];pzmap(num,den)
% num=[0.5 5];
% d1=[0.5 1];d2=[1 0.6/5 1];
% den=conv(d1,d2);
% subplot(1,2,1)
% bode(num,den)
% subplot(1,2,2)
% nyquist(num,den)

% num=1;den=[1 2 2];
% [mag,phase,w]= bode(num,den);
% bode(num,den)
% grid on
% figure;
% subplot(1,2,1)
% plot(w,mag,'r')
% subplot(1,2,2)
% plot(w,phase,'g')
% num = 1;
% den = [1, 2, 2];
% G = tf(num, den);
% 
% % 计算增益裕度、相位裕度，并绘制Bode图
% [Gm, Pm, Wcg, Wcp] = margin(G);

% num=1;den=[conv([1,0],conv([1,4],conv([1 2-4i],[1 2+4i])))];
% G=tf(num,den);
% rlocus(G)
% [r,k]=rlocus(G)%闭环极点和增益
% [k,p]=rlocfind(G)%定位点的极点和增益
% sgrid(0.707,10)

clear,clc;
T=0.01;
Kp=1.8;Ki=1.5;Kd=0.5;
t=0:T:20;
e=0;u=0;y=0;
r=ones(1,length(t));
ierror=0;derror=0;


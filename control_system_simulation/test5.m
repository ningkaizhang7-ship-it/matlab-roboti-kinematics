s = tf('s');
K = 200;
G = K / (s*(4*s + 10));

margin(G);

alpha = 0.1;
omega_m = 4;  
T = 1 / (omega_m * sqrt(alpha));

G_corr = tf([T 1], [alpha*T 1]);


figure;
margin(G); hold on;
margin(G_corr);
legend('未校正', '超前校正');
title('校正前后 Bode 图对比');
grid on;

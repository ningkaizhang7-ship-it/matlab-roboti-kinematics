figure;
%%改进D-H模型
%       theta    d           a        alpha     offset
SL1=Link([0      0           0        0           0     ],'modified');
SL2=Link([0      0           0        -pi/2       0     ],'modified');
SL3=Link([0      0.149       0.432    0           0     ],'modified');
SL4=Link([0      0.433       0.02     -pi/2       0     ],'modified');
SL5=Link([0      0           0        pi/2        0     ],'modified');
SL6=Link([0      0           0        -pi/2       0     ],'modified');
p560=SerialLink([SL1 SL2 SL3 SL4 SL5 SL6],'name','puma560');
p560.teach([0 0 0 0 0 0]);

figure;

%%标准D-H模型
%       theta    d           a        alpha     offset
SL1=Link([0      0           0        -pi/2       0     ],'standard');
SL2=Link([0      0           0.432     0          0     ],'standard');
SL3=Link([0      0.149       0.02    -pi/2        0     ],'standard');
SL4=Link([0      0.433       0        pi/2        0     ],'standard');
SL5=Link([0      0           0        -pi/2       0     ],'standard');
SL6=Link([0      0           0        0           0     ],'standard');
p560=SerialLink([SL1 SL2 SL3 SL4 SL5 SL6],'name','puma560');
p560.teach([0 0 0 0 0 0]);




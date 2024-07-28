syms Q_in A_g A_a T_air T_sea K_a K_g h_w h_a L_a L_g A_d K_d L_d L R_o R_i

%REEFSCAN TRANSOM DIMENSIONS

Q_in = 4.5; % 23.9; % 
A_g = 0.011423;
% T_air = 39.3;
T_sea = 30;
K_a = 235;
K_g = 1.38;
K_d = 0.37;
h_w = 50;
h_a = 3.55;
L_a = 0.015; % 0.008; %
L_g = 0.004;
L_d = 0.01;
L = 0.164;
R_o = 0.09/2;
R_i = 0.07/2;


A_tube = 0.0349; % 0.122;
A_cap =  0.00319; % 0.0078; % 

% A_a = A_cap;
A_d = A_tube;

A_ai = 0.003185;
A_ao= 0.005404;


eqn = Q_in == (A_g*1/(1/h_w+L_g/K_g+1/h_a) + 1/(1/(A_ao*h_w)+L_a/(0.5*(A_ao-A_ai)*K_a)+1/(A_ai*h_a)) + 1/(1/(h_a*2*pi*R_i*L)+(log(R_o/R_i)/(2*pi*K_d*L)+1/(h_w*2*pi*R_o*L))))*(T_air-T_sea);

soln =(solve(eqn,T_air));

soln = round(soln,6)
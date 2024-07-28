syms Q_in
syms T_air T_di T_do T_gi T_go T_ai T_ao T_sea T_board
syms h_air h_water
syms k_d k_g k_a
syms A_g A_sink A_ai A_ao
syms r_i r_o
syms L_d L_g L_a L_sink
syms L_tot
syms Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 Q9 Q10

Q_in = 23.9;% W
T_sea = 30; % Celsius
h_air = 3.5;% W/m^2K
h_water = 500;% W/m^2K
k_d = 0.37;% W/mK
k_g = 1.38;% W/mK
k_a = 235;% W/mK
A_g = 0.011423; % m^2
A_sink = 0.00018;% m^2
A_ai = 0.003185;% m^2
A_ao= 0.005404;% m^2
r_i = 0.078/2;% m
r_o = 0.1/2;% m
L_d = 0.01;% m
L_g = 0.005;% m
L_a = 0.015;% m
L_sink = 0.054;% m
L_tot = 0.25;% m


% T_board = 60;

S = solve([T_board == T_air + 75,
Q1 == (h_air*2*pi*r_i*L_tot)*(T_air-T_di),
Q2 == (h_air*A_g)*(T_air-T_gi),
Q3 == (h_air*(A_ai-A_sink))*(T_air-T_ai),
Q4 == (2*pi*k_d*L_tot/log(r_o/r_i))*(T_di-T_do),
Q5 == (k_g/L_g*A_g)*(T_gi-T_go),
Q6 == (0.5*(A_ao+A_ai)*k_a/L_a)*(T_ai-T_ao),
Q7 == (h_water*2*pi*r_o*L_tot)*(T_do-T_sea),
Q8 == (h_water*A_g)*(T_go-T_sea),
Q9 == (h_water * A_ao)*(T_ao-T_sea),
Q10 == (k_a*A_sink/L_sink)*(T_board-T_ai),
Q1+Q2+Q3+Q10 == Q_in,
Q4 == Q1,
Q5 == Q2,
Q3+Q10 == Q6,
Q4 == Q7,
Q5 == Q8,
Q6 == Q9, 
Q7+Q8+Q9 == Q_in]);




T_air_soln = round(S.T_air, 6)
% sol = round(S.T_board, 6)
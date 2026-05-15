%% Autonomous Tomato Harvester Control System
clc; clear; close('all');

g = 9.81; % [m/s^2]
n = 184; % gear ratio

m_act_base = .591; % [kg]
m_act_rod = 0.154; % [kg]
m_act_screw = 0.41; % [kg]
m_act_case = 0.209; % [kg]
m_load = 0; % [kg] include??
m_act = m_act_base + m_act_rod + m_act_screw + m_act_case + m_load; % [kg]

l_act_const = 0.550; % [m]
l_act_rod = 0.511175; % [m]

%Operating Points
x1_o = 0;
x2_o = 1; % [rad/s] - phi'
x3_o = 4.5; % [A] - i_1
x4_o = pi/4; % [rad] - theta
x5_o = .5; % [rad/s] - theta'
x6_o = 4.5; % [A] - i_2
x7_o = .4572/2; % [m] - delta r
x8_o = .01;
x9_o = 2;

pulse = 110;

%%MOTOR 1: PHI
J_r1 = 0.003; 
J_r2 = 1.25*10^-6;
J_column_phi = 0.25; % [kg*m^2]
J_m1out = (n^2)*J_r1 + J_r2 + J_column_phi;
J_load_phi = 0;
J_const_phi = J_m1out + J_load_phi;
%Actuator MOI f(theta) BUT actually theta and r
J_act_horiz = 0.34656; % CAD GUESS
J_act_vert = .005; % CAD GUESS
J_act_const_phi = (J_act_vert + J_act_horiz)/2;
J_act_var_phi = (J_act_vert - J_act_horiz)/2; % [kg*m^2]

R_a1 = 1.36; % [Ohm]
K_r1 = .11; % [Nm/A] or [Nm*s/rad]
L_a1 = 0.3; % [H]
b_r1 = .0013796776;
b_r2 = .11; %bearing & output shaft damping
b_load_phi = 0;
b_eq_phi = (n^2)*b_r1 + b_r2 + b_load_phi; % [Nm*s/rad]

%%MOTOR 2: THETA
J_r1 = 0.003; 
J_r2 = 1.25*10^-6;
J_column_theta = 0.451; % shafts, couplers, adapter, encoder MOI
J_m2out = (n^2)*J_r1 + J_r2 + J_column_theta;
J_const_theta = J_m2out;
%Actuator MOI f(r)
J_act_const_theta = ((m_act_base + m_act_case + m_act_screw)*(.5588^2) + m_act_rod*(.508^2))/3;


r_cg_const = (l_act_const*.5*(m_act_case + m_act_screw) + .06*m_act_base)/m_act; % [m]
r_cg_var = ((.4572 + l_act_rod*.5)*m_act_rod + (.4572 + l_act_const)*m_load)/m_act; % [m]
r_cg = r_cg_const + r_cg_var;

R_a2 = 1.36; % [ohm]
K_r2 = .11; % [Nm/A] or [Nm*s/rad]
L_a2 = 0.3; % [H]
b_r1 = .0013796776;
b_r2 = .11; %bearing & output shaft damping
b_load_theta = 0;
b_eq_theta = (n^2)*b_r1 + b_r2 + b_load_theta; % [Nm*s/rad]

%%MOTOR 3: r
b_eq_r = 0.06;
J_eq_r = 0.0018;
p_s = 0.0035;
n_3 = 3.07500;
K_r3 = 0.0709;
L_a3 = 0.09;
R_a3 = 2.0500;

% x2 linearized
syms x2 x3 x4;
x2_dot = (n*K_r1*x3 - b_eq_phi*x2)/((J_const_phi + J_act_const_phi + (J_act_var_phi*cos(2*x4)))); %ODE

x2_partial_x2 = diff(x2_dot, x2);
double(subs(x2_partial_x2,x4,x4_o));
x2_partial_x3 = diff(x2_dot, x3);
double(subs(x2_partial_x3,x4,x4_o));
x2_partial_x4 = diff(x2_dot, x4);
double(subs(x2_partial_x4,[x2 x3 x4],[x2_o x3_o x4_o]));

% x5 linearized
syms x4 x5 x6 x7
x5_dot = (m_act*g*(r_cg_const + ((x7 + l_act_rod*.5)*m_act_rod + (x7 + l_act_const)*m_load)/m_act)*sin(x4) - b_eq_theta*x5 + n*K_r2*x6)/(J_const_theta + J_act_const_theta + m_act_rod*(x7)^2); %ODE

x5_partial_x4 = diff(x5_dot, x4);
double(subs(x5_partial_x4,[x4 x7],[x4_o x7_o]));
x5_partial_x5 = diff(x5_dot, x5);
double(subs(x5_partial_x5,x7,x7_o));
x5_partial_x6 = diff(x5_dot, x6);
double(subs(x5_partial_x6,x7,x7_o));
x5_partial_x7 = diff(x5_dot, x7);
double(subs(x5_partial_x7,[x4 x5 x6 x7],[x4_o x5_o x6_o x7_o]));

%State Space Matrices
A = [0, 1, 0, 0, 0, 0, 0, 0, 0; 
    0, double(subs(x2_partial_x2,x4,x4_o)), double(subs(x2_partial_x3,x4,x4_o)), double(subs(x2_partial_x4,[x2 x3 x4],[x2_o x3_o x4_o])), 0, 0, 0, 0, 0;
    0, -n*K_r1/L_a1, -R_a1/L_a1, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 1, 0, 0, 0, 0;
    0, 0, 0, double(subs(x5_partial_x4,[x4 x7],[x4_o x7_o])), double(subs(x5_partial_x5,x7,x7_o)), double(subs(x5_partial_x6,x7,x7_o)), double(subs(x5_partial_x7,[x4 x5 x6 x7],[x4_o x5_o x6_o x7_o])), 0, 0;
    0, 0, 0, 0, -n*K_r2/L_a2, -R_a2/L_a2, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, (m_act_rod + m_load)*g*sin(x4_o)*(p_s)^2/(4*(pi^2)*J_eq_r), 0, 0, 0, -b_eq_r/J_eq_r, n_3*K_r3*p_s/(2*pi*J_eq_r);
    0, 0, 0, 0, 0, 0, 0, -2*pi*n_3*K_r3/(p_s*L_a3), -R_a3/L_a3];

B = [0, 0, 0;
     0, 0, 0;
     1/L_a1, 0, 0;
     0, 0, 0;
     0, 0, 0;
     0, 1/L_a2, 0;
     0, 0, 0;
     0, 0, 0;
     0, 0, 1/L_a3];
C = [1 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0];
D = [0 0 0;
     0 0 0;
     0 0 0];

sys = ss(A,B,C,D);
sysPoles = eig(A);
disp('The Open Loop System has poles:');
disp(sysPoles)

% Check controllability
Cm = ctrb(A, B);
rank_Cm = rank(Cm);
fprintf('Rank of the Controllability Matrix: %d\n', rank_Cm);
uC = size(A, 1) - rank_Cm;
fprintf('The System has <strong>%d</strong> Uncontrollable States.\n', uC);

% Check observability
Om = obsv(A, C);                                                                                                         
rank_Om = rank(Om);
fprintf('Rank of the Observability Matrix: %d\n', rank_Om);
uO = size(A, 1) - rank_Om;
fprintf('The System has <strong>%d</strong> Unobservable States.\n\n', uO);

Q = diag([500 1 1 500 1 1 10000 1 .1]);
R = diag([5 10 .005]);

K = lqr(A,B,Q,R);
disp('State Feedback Gains (K):');
disp(K);
Kr = inv(C*inv(-A+B*K)*B);
sysCL = ss(A-B*K,B,C,D);

% tomato_positions = [-1*phi 0; theta-2 0; (r*2.54)-59.639 0];
tomato_positions = [34; 60; 12.5];
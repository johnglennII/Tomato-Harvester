%% Screw Linear Actuator System Identification
clc; clear; close('all');

load actuatorData.mat

actuatorData.OutputName = 'Position';
actuatorData.OutputUnit = 'm';
actuatorData.Tstart = 0;
actuatorData.TimeUnit = 's';

%Ts = .01;
% Linear actuator model (general form)
function [A,B,C,D] = actuator(b_eq_r, J_eq_r, p_s, n_3, K_r3, L_a3, R_a3, Ts)
A = [0, 1, 0;
     0, -b_eq_r/J_eq_r, p_s*n_3*K_r3/(2*pi*J_eq_r);
     0, -n_3*K_r3*2*pi/(p_s*L_a3), -R_a3/L_a3];
B = [0; 0; 1/L_a3];
C = [1, 0, 0];
D = 0;
end

% g = 9.81;
n_3 = 45/12; % gear ratio (fixed/known)
p_s = .0035; % screw lead [m] (fixed/known)
J_eq_r = 0.014077900000000; % (free/unknown)

R_a3 = 1.23/.6; % [Ohm] (fixed/known)
K_r3 = .18; % [Nm/A] or [Nm*s/rad] (free/unknown)
L_a3 = 0.1; % [H] (free/unknown)
b_eq_r = 0.023906250000000; % (free/unknown)

linear_model = idgrey(@actuator, {b_eq_r, J_eq_r, p_s, n_3, K_r3, L_a3, R_a3}, 'c');
Labels = {'b_eq_r'; 'J_eq_r'; 'p_s'; 'n_3'; 'K_r3'; 'L_a3'; 'R_a3'};

linear_model.Structure.Parameters(3).Free = false;
linear_model.Structure.Parameters(4).Free = false;
linear_model.Structure.Parameters(7).Free = false;

opt = greyestOptions('InitialState','estimate','Display','on');
opt.EnforceStability = true;

linear_model = greyest(actuatorData2,linear_model,opt);

estimatedParams = linear_model.Report.Parameters.ParVector;
disp('Estimated Parameters:');
for i = 1:length(Labels)
    fprintf('%s: %.4f\n', Labels{i,1}, estimatedParams(i,1));
end

compare(actuatorData2, linear_model)

%% Motor SYS ID
clc; clear; close('all');

load motorData.mat

motorData.OutputName = 'Angular Position';
motorData.OutputUnit = 'rad';
motorData.Tstart = 0;
motorData.TimeUnit = 's';

%Ts = .01;
function [A,B,C,D] = motor(b_eq_phi, J_const_phi, n, K_r1, L_a1, R_a1, Ts)
A = [0, 1, 0;
     0, -b_eq_phi/J_const_phi, n*K_r1/J_const_phi;
     0, -n*K_r1/L_a1, -R_a1/L_a1];
B = [0; 0; 1/L_a1];
C = [1, 0, 0];
D = 0;
end

n = 184;
J_const_phi = 101.5680;
R_a1 = 1.36; % [Ohm]
K_r1 = .11; % [Nm/A] or [Nm*s/rad]
L_a1 = 0.3; % [H]
b_eq_phi = 46.8204;

linear_model = idgrey(@motor, {b_eq_phi, J_const_phi, n, K_r1, L_a1, R_a1}, 'c');
Labels = {'b_eq_phi'; 'J_const_phi'; 'n'; 'K_r1'; 'L_a1'; 'R_a1'};

%Known Parameters
linear_model.Structure.Parameters(3).Free = false;

opt = greyestOptions('InitialState','estimate','Display','on');
opt.EnforceStability = true;

linear_model = greyest(motorData,linear_model,opt);

estimatedParams = linear_model.Report.Parameters.ParVector;
disp('Estimated Parameters:');
for i = 1:6
    fprintf('%s: %.4f\n', Labels{i,1}, estimatedParams(i,1));
end

compare(motorData, linear_model)
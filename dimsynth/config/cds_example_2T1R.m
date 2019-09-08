% Einstellungen für komb. Struktur und Maßsynthese für 2T1R Mechanismen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

% Aufgaben-FG
DoF = [1 1 0 0 0 1];
Traj_no = 1;

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;

Traj = cds_gen_traj(DoF, Traj_no, Set.task);

Set.optimization.objective = 'mass';
Set.optimization.optname = '2T1R_test';
Set.optimization.NumIndividuals = 5;
Set.optimization.MaxIter = 5;
Set.general.plot_details_in_fitness = 1e3;
Set.general.plot_robot_in_fitness = 1e3;
Set.structures.whitelist = {'P3RRR1A1'}; % '' S3RRR1

Set.task.payload = struct('m', 0, 'rS', zeros(3,1), 'Ic', zeros(6,1));
cds_start

return
% Beispiel für Fitness-Funktion 3RRR
resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
tmp = load(fullfile(resmaindir, ...
  sprintf('Rob%d_%s', 1, 'P3RRR1A1_Endergebnis.mat')), 'RobotOptRes', 'Set', 'Traj');
% tmp.RobotOptRes.vartypes
tmp.RobotOptRes.fitnessfcn(tmp.RobotOptRes.p_val)
% disp(tmp.RobotOptRes.p_val)
% 
x_test = tmp.RobotOptRes.p_val(:);
disp(tmp.RobotOptRes.p_types);
disp(tmp.RobotOptRes.p_limits);

for i = 1:length(tmp.RobotOptRes.p_val)
  fprintf('p%d: %20s; type %d; min %+1.2f, max %+1.2f, opt %1.2f\n', ...
    i, tmp.RobotOptRes.Structure.varnames{i}, tmp.RobotOptRes.p_types(i), ...
    tmp.RobotOptRes.p_limits(i,1), tmp.RobotOptRes.p_limits(i,2), tmp.RobotOptRes.p_val(i));
end

x_test(tmp.RobotOptRes.p_types==0) = 0.7; % Allgemeine Skalierung
% x_test(tmp.RobotOptRes.p_types==2) = [0.5;0.5]; % Basis-Position
x_test(tmp.RobotOptRes.p_types==1) = [0.8; 0.8]; % Beinlängen
x_test(tmp.RobotOptRes.p_types==7) = 0.01; % Plattform-Radius
x_test(tmp.RobotOptRes.p_types==6) = 0.07; % Gestell-Radius
% Setze Basis-Position auf Mitte
x_test(tmp.RobotOptRes.p_types==2) = ...
  mean(tmp.RobotOptRes.p_limits(tmp.RobotOptRes.p_types==2,:)')';

tmp.Set.general.plot_robot_in_fitness = Inf;
cds_dimsynth_fitness_par(tmp.RobotOptRes.R,tmp.Set,tmp.Traj,tmp.RobotOptRes.Structure,x_test)


tmp.RobotOptRes.fitnessfcn(x_test)
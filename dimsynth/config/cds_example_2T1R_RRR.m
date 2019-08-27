% Einstellungen für komb. Struktur und Maßsynthese für 2T1R Mechanismen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

% Aufgaben-FG
DoF = [1 1 0 0 0 1];

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
Set.optimization.NumIndividuals = 100;
Set.optimization.MaxIter = 25;
Set.general.plot_details_in_fitness = 1e3;
Set.general.plot_robot_in_fitness = 1e3;
Set.structures.whitelist = {'P3RRR1A1', 'S3RRR1'};
for Traj_no = 1:2
  for obj_name = {'mass', 'energy', 'condition'}
    Traj = cds_gen_traj(DoF, Traj_no, Set.task);
    Set.optimization.objective = obj_name;
    Set.optimization.optname = sprintf('2T1R_test_RRR_T-%d_O-%s', Traj_no, obj_name{1});
    cds_start
  end
end
% Einstellungen für komb. Synthese für 3T2R-Aufgabenredundanz
% Minimalbeispiel, das den Effekt zeigt und Fehler aufdeckt.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Aufgaben-FG
DoF = [1 1 1 1 1 0];
Traj_no = 1;

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;

% Schwenkwinkel etwas verringern (ist sonst zu schwer erreichbar und das
% hier ist ja nur ein Testskript)
Set.task.maxangle = 30*pi/180;
Traj = cds_gen_traj(DoF, Traj_no, Set.task);

%% Debug: Kompilieren und Erstellen von Funktionen
% Seriell:
% serroblib_create_template_functions({'S6RRRRRR10V2'},  false, false);
% serroblib_addtopath({'S6RRRRRR10V2'});
% matlabfcn2mex({['S6RRRRRR10V2','_invkin_eulangresidual']});
% matlabfcn2mex({['S6RRRRRR10V2','_invkin_traj']});
% Parallel:
% serroblib_create_template_functions({'S6PRRRRR6V4'},  false, false);
% parroblib_create_template_functions({'P6PRRRRR6V4G8P1A1'}, false, false);
% parroblib_addtopath({'P6PRRRRR6V4G8P1A1'});
% matlabfcn2mex({['P6PRRRRR6V4','_invkin']});
% matlabfcn2mex({['P6PRRRRR6V4','_invkin3']});
% matlabfcn2mex({['P6PRRRRR6V4','_invkin_traj']});

% Zur Generierung in Synthese (erzeugt nicht die mex-Dateien neu)
% Set.general.create_template_functions = true;
%% Weitere Einstellungen
Set.optimization.objective = {'condition', 'chainlength'};
Set.optimization.optname = '3T2R_taskred_test';
Set.optimization.NumIndividuals = 50;
Set.optimization.MaxIter = 20;
Set.optimization.obj_limit = [1e3;1e3]; % Bei erstem Erfolg aufhören
Set.optimization.max_range_passive_spherical = 240*pi/180; % Technisch unrealistisch, aber für Ergebnis-Veranschaulichung besser.
Set.general.plot_robot_in_fitness = 0;%1e3; % Bei Erfolg
Set.general.plot_details_in_fitness = 0;%4e9; % Debug-Plots für Selbstkollision
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 3;
Set.optimization.ee_rotation = true; % TODO: EE-Rotation müsste noch zur Plausibilität begrenzt werdne
Set.optimization.ee_translation = false;
Set.structures.max_task_redundancy = 1;
Set.optimization.constraint_collisions = true; %für plausible Ergebnisse und Nullraumbewegung)
Set.general.eval_figures = {};
Set.general.animation_styles = {};
Set.general.debug_calc = true;

% Wähle vier verschiedene Strukturen: 3T2R/3T3R, Seriell/Parallel
Set.structures.whitelist = {'S6RRRRRR10V2', 'S5RRRRR1', ...
  'P5RPRRR8V1G9P8A1', 'P6PRRRRR6V4G8P1A1'};
cds_start

% Nachträglich das Pareto-Diagramm zeichnen (zum Debuggen)
return
Set = cds_settings_defaults(struct('DoF', [1 1 1 1 1 0])); %#ok<UNRCH>
Traj = [];
% Debug: Nur Tabelle neu generieren:
Set.general.regenerate_summary_only = true;
Set.general.eval_figures = {'pareto_all_phys'};
Set.general.animation_styles = {};
Set.general.parcomp_plot = false;
Set.optimization.optname = '3T2R_taskred_test';
cds_start

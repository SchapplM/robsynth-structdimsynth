% Einstellungen für komb. Struktur und Maßsynthese mit Bauraumgrenze
% Szenario: Griff von oben in eine Tonne. Vermeide Kollisionen mit Umgebung

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Funktion für Kollisionsprüfung kompilieren (falls noch nicht getan):
% matlabfcn2mex({'check_collisionset_simplegeom'})

% Aufgaben-FG
DoF = [1 1 1 0 0 0];
Traj_no = 1;

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
% Bauraum definieren
Set.task.installspace = struct( ...
  'type', uint8([1; 2]), ...
  'params', [[[-1.0, -1.0, 0.5], [2.0,0,0], [0,2.0,0], 2.0]; ... % Quader oberhalb der Aufgabe
             [[0,0,-0.1], [0,0,0.5], 0.5, NaN(1,3)]], ... % Zylinder Höhe 600mm, Radius 500mm
  'links', {{0:1; 1:6}});  % Im Quader dürfen nur die Basis und das erste bewegte Segment sein. Im Zylinder müssen alle bewegten Segmente sein.
% Hindernisse definieren: Objekte für Übergang zwischen Zylinder und Quader
% (damit der Roboter nicht schräg vom Quader zum Zylinder durchgreift)
% Erzeuge quadratische Sperre aus Kapseln
Set.task.obstacles = struct( ...
  'type', repmat(uint8(3),4,1), ... % Kapseln (nichts anderes implementiert)
  'params', [[[-0.6 -0.6 0.4], [-0.6  0.6 0.4], 0.10]; ...
             [[ 0.6 -0.6 0.4], [ 0.6  0.6 0.4], 0.10]; ...
             [[-0.6 -0.6 0.4], [ 0.6 -0.6 0.4], 0.10]; ...
             [[-0.6  0.6 0.4], [ 0.6  0.6 0.4], 0.10]]);

Traj = cds_gen_traj(DoF, Traj_no, Set.task);
% Ziehe Trajektorie (ursprünglich Mittelpunkt [0.5 0.5 0]) auf [0 0 0.1]
% Trajektorie muss in Bauraum liegen
X_offset = [0.65 0.35 -0.3];
Traj.XE(:,1:3) = Traj.XE(:,1:3)-repmat(X_offset, size(Traj.XE,1),1);
Traj.X(:,1:3) =  Traj.X(:,1:3)- repmat(X_offset, size(Traj.X, 1),1);
Set.optimization.objective = 'mass';
Set.optimization.optname = 'installspace_test';
Set.optimization.NumIndividuals = 50;
Set.optimization.MaxIter = 20;
Set.general.plot_details_in_fitness = 1e7; % Debug-Plots für Kollisionen/Bauraum
Set.general.plot_robot_in_fitness = 1e3;
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 3;
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig'};
Set.optimization.constraint_collisions = false;
% Das Gestell sollte nicht größer als die Bauraumgrenze (Quader) sein
Set.optimization.base_size_limits = [0.1, 1.0*sqrt(2)];
% Die Plattform sollte nicht größer als die Bauraumgrenze (Zylinder) sein
Set.optimization.platform_size_limits = [0.05, 0.2];
% Die Basis muss innerhalb der oberen Bauraumgrenze (Quader) sein
Set.optimization.basepos_limits = [[NaN, NaN]; [NaN, NaN]; [0.5 1.5]]; % nur z-Komponente
Set.structures.maxnumprismatic = 3; % für Portal-Systeme (trotzdem ohne passive Schubgelenke)
% Mögliche Kandidaten, die halbwegs gut für obigen Aufgabe funktionieren:
% P3RRRRR6G2P2A1, P3RRRRR7G3P3A1, P3RRRRR10G3P3A1, P3RRRRR10G2P2A2, S3PPP1
Set.structures.whitelist = {'P3PRRRR6G4P2A1'};
cds_start
